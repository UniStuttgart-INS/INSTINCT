// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LowPassFilter.hpp"

#include "NodeRegistry.hpp"
#include <algorithm>
#include <imgui.h>
#include "Navigation/INS/Units.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/GNSS/Functions.hpp"

#include "util/Eigen.hpp"
#include "util/StringUtil.hpp"
#include "util/Logger.hpp"

#include <imgui_internal.h>
#include <limits>
#include <set>
#include <type_traits>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), ImuObsWDelta::type() };

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::LowPassFilter::LowPassFilter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 500, 300 };

    nm::CreateInputPin(this, "True", Pin::Type::Flow, supportedDataIdentifier, &LowPassFilter::receiveObs);

    nm::CreateOutputPin(this, "Biased", Pin::Type::Flow, supportedDataIdentifier);
}

NAV::LowPassFilter::~LowPassFilter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LowPassFilter::typeStatic()
{
    return "LowPassFilter";
}

std::string NAV::LowPassFilter::type() const
{
    return typeStatic();
}

std::string NAV::LowPassFilter::category()
{
    return "Data Processor";
}

void NAV::LowPassFilter::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        ImGui::TextUnformatted("Filter Type:");

        ImGui::SetNextItemWidth(columnWidth);
        if (ImGui::BeginCombo(fmt::format("##{}", size_t(id)).c_str(), to_string(_filterType)))
        {
            for (size_t i = 0; i < static_cast<size_t>(FilterType::COUNT); i++)
            {
                const bool is_selected = (static_cast<size_t>(_filterType) == i);
                if (ImGui::Selectable(to_string(static_cast<FilterType>(i)), is_selected))
                {
                    _filterType = static_cast<FilterType>(i);
                    LOG_DEBUG("{}: filterType changed to {}", nameId(), fmt::underlying(_filterType));
                    flow::ApplyChanges();
                    doDeinitialize();
                }
                if (is_selected) // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        if (_filterType == FilterType::Linear)
        {
            const float ITEM_WIDTH = 100.0F * NodeEditorApplication::defaultFontRatio();
            ImGui::TextUnformatted("Filter properties:");
            ImGui::SetNextItemWidth(ITEM_WIDTH);
            if (ImGui::InputDoubleL(fmt::format("Cutoff Frequency Accelerometer##{}", size_t(id)).c_str(), &_linear_filter_cutoff_frequency_accel, 1e-5, 1e5, 0.0, 0.0, "%.5f Hz"))
            {
                LOG_DEBUG("{}: Cutoff Freq. Accel changed to {}", nameId(), _linear_filter_cutoff_frequency_accel);
                flow::ApplyChanges();
                doDeinitialize();
            }

            ImGui::SetNextItemWidth(ITEM_WIDTH);
            if (ImGui::InputDoubleL(fmt::format("Cutoff Frequency Gyroscope##{}", size_t(id)).c_str(), &_linear_filter_cutoff_frequency_gyro, 1e-5, 1e5, 0.0, 0.0, "%.5f Hz"))
            {
                LOG_DEBUG("{}: Cutoff Freq. Gyro changed to {}", nameId(), _linear_filter_cutoff_frequency_gyro);
                flow::ApplyChanges();
                doDeinitialize();
            }
        }
    }
}

json NAV::LowPassFilter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["filterType"] = _filterType;
    j["linear_filter_cutoff_frequency_accel"] = _linear_filter_cutoff_frequency_accel;
    j["linear_filter_cutoff_frequency_gyro"] = _linear_filter_cutoff_frequency_gyro;

    return j;
}

void NAV::LowPassFilter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());
    if (j.contains("filterType")) { j.at("filterType").get_to(_filterType); }
    if (j.contains("linear_filter_cutoff_frequency_accel")) { j.at("linear_filter_cutoff_frequency_accel").get_to(_linear_filter_cutoff_frequency_accel); }
    if (j.contains("linear_filter_cutoff_frequency_gyro")) { j.at("linear_filter_cutoff_frequency_gyro").get_to(_linear_filter_cutoff_frequency_gyro); }
}

bool NAV::LowPassFilter::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        DataToFilter_Accel.clear();
        DataToFilter_Gyro.clear();
    }

    return true;
}

void NAV::LowPassFilter::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id)
    {
        return; // Link on Output Port
    }

    // Store previous output pin identifier
    auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
    // Overwrite output pin identifier with input pin identifier
    outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin.dataIdentifier;

    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsWDelta::type() }))
    {
        _inputType = InputType::ImuObsWDelta;
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObs::type() }))
    {
        _inputType = InputType::ImuObs;
    }

    if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
    {
        // Check if connected links on output port are still valid
        for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
        {
            if (auto* endPin = link.getConnectedPin())
            {
                if (!outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
                {
                    // If the link is not valid anymore, delete it
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
                }
            }
        }

        // Refresh all links connected to the output pin if the type changed
        if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
        {
            for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
            {
                if (auto* connectedPin = link.getConnectedPin())
                {
                    outputPins.at(OUTPUT_PORT_INDEX_FLOW).recreateLink(*connectedPin);
                }
            }
        }
    }
}

void NAV::LowPassFilter::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if ((endPin.parentNode->id != id                                  // Link on Output port is removed
         && !inputPins.at(INPUT_PORT_INDEX_FLOW).isPinLinked())       //     and the Input port is not linked
        || (startPin.parentNode->id != id                             // Link on Input port is removed
            && !outputPins.at(OUTPUT_PORT_INDEX_FLOW).isPinLinked())) //     and the Output port is not linked
    {
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = supportedDataIdentifier;
    }
}

void NAV::LowPassFilter::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = queue.extract_front();

    // #########################################################################################################################################

    // Select the correct data type and make a copy of the node data to modify
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsSimulated::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW,
                        receiveImuObsWDelta(std::make_shared<ImuObsSimulated>(*std::static_pointer_cast<const ImuObsSimulated>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObsWDelta::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW,
                        receiveImuObsWDelta(std::make_shared<ImuObsWDelta>(*std::static_pointer_cast<const ImuObsWDelta>(obs))));
    }
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { ImuObs::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW,
                        receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(obs))));
    }
}

std::shared_ptr<NAV::ImuObs> NAV::LowPassFilter::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    if (_filterType == FilterType::Linear)
    {
        return FitLinearTrend(imuObs);
    }

    return imuObs;
}

std::shared_ptr<NAV::ImuObsWDelta> NAV::LowPassFilter::receiveImuObsWDelta(const std::shared_ptr<ImuObsWDelta>& imuObsWDelta)
{
    if (_filterType == FilterType::Linear)
    {
        return FitLinearTrend(imuObsWDelta);
    }

    return imuObsWDelta;
}

std::shared_ptr<NAV::ImuObs> NAV::LowPassFilter::FitLinearTrend(const std::shared_ptr<ImuObs>& imuObs)
{
    // first we filter accelerations
    DataToFilter_Accel[imuObs->insTime] = imuObs->p_acceleration;
    // for testing at the moment
    double dt_accel = 1.0 / _linear_filter_cutoff_frequency_accel;
    // remove all entries that are outside filter time window
    std::erase_if(DataToFilter_Accel, [&](const auto& pair) { return static_cast<double>((imuObs->insTime - pair.first).count()) > dt_accel; });

    if (DataToFilter_Accel.size() > 2)
    {
        // average accelerations first
        auto N11 = static_cast<double>(DataToFilter_Accel.size());
        double N12 = 0.0;
        double N22 = 0.0;
        Eigen::VectorXd n1 = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd n2 = Eigen::VectorXd::Zero(3);
        for (const auto& key_val : DataToFilter_Accel)
        {
            double delta_t = static_cast<double>((key_val.first - imuObs->insTime).count());
            N12 += delta_t;
            N22 += delta_t * delta_t;
            n1 += key_val.second;
            n2 += delta_t * key_val.second;
        }
        double determinant_inverse = 1.0 / (N11 * N22 - N12 * N12);
        Eigen::VectorXd filtered = determinant_inverse * (N22 * n1 - N12 * n2);
        imuObs->p_acceleration = filtered;
    }

    // then we filter gyro data
    DataToFilter_Gyro[imuObs->insTime] = imuObs->p_angularRate;

    double dt_gyro = 1.0 / _linear_filter_cutoff_frequency_gyro;
    // remove all entries that are outside filter time window
    std::erase_if(DataToFilter_Gyro, [&](const auto& pair) { return static_cast<double>((imuObs->insTime - pair.first).count()) > dt_gyro; });
    if (DataToFilter_Gyro.size() > 2)
    {
        // average accelerations first
        auto N11 = static_cast<double>(DataToFilter_Gyro.size());
        double N12 = 0.0;
        double N22 = 0.0;
        Eigen::VectorXd n1 = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd n2 = Eigen::VectorXd::Zero(3);
        for (const auto& key_val : DataToFilter_Gyro)
        {
            double delta_t = static_cast<double>((key_val.first - imuObs->insTime).count());
            N12 += delta_t;
            N22 += delta_t * delta_t;
            n1 += key_val.second;
            n2 += delta_t * key_val.second;
        }
        double determinant_inverse = 1.0 / (N11 * N22 - N12 * N12);
        Eigen::VectorXd filtered = determinant_inverse * (N22 * n1 - N12 * n2);
        imuObs->p_angularRate = filtered;
    }

    return imuObs;
}

std::shared_ptr<NAV::ImuObsWDelta> NAV::LowPassFilter::FitLinearTrend(const std::shared_ptr<ImuObsWDelta>& imuObsWDelta)
{
    // first we filter accelerations
    Eigen::VectorXd v1(6);
    v1 << imuObsWDelta->p_acceleration, imuObsWDelta->dvel;
    DataToFilter_Accel[imuObsWDelta->insTime] = v1;
    // for testing at the moment
    double dt_accel = 1.0 / _linear_filter_cutoff_frequency_accel;
    // remove all entries that are outside filter time window
    std::erase_if(DataToFilter_Accel, [&](const auto& pair) { return static_cast<double>((imuObsWDelta->insTime - pair.first).count()) > dt_accel; });

    if (DataToFilter_Accel.size() > 2)
    {
        // average accelerations first
        auto N11 = static_cast<double>(DataToFilter_Accel.size());
        double N12 = 0.0;
        double N22 = 0.0;
        Eigen::VectorXd n1 = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd n2 = Eigen::VectorXd::Zero(6);
        for (const auto& key_val : DataToFilter_Accel)
        {
            double delta_t = static_cast<double>((key_val.first - imuObsWDelta->insTime).count());
            N12 += delta_t;
            N22 += delta_t * delta_t;
            n1 += key_val.second;
            n2 += delta_t * key_val.second;
        }
        double determinant_inverse = 1.0 / (N11 * N22 - N12 * N12);
        Eigen::VectorXd filtered = determinant_inverse * (N22 * n1 - N12 * n2);
        imuObsWDelta->p_acceleration = filtered.segment(0, 3);
        imuObsWDelta->dvel = filtered.segment(3, 3);
    }

    // then we filter gyro data
    Eigen::VectorXd v2(6);
    v2 << imuObsWDelta->p_angularRate, imuObsWDelta->dtheta;
    DataToFilter_Gyro[imuObsWDelta->insTime] = v2;

    double dt_gyro = 1.0 / _linear_filter_cutoff_frequency_gyro;

    // remove all entries that are outside filter time window
    std::erase_if(DataToFilter_Gyro, [&](const auto& pair) { return static_cast<double>((imuObsWDelta->insTime - pair.first).count()) > dt_gyro; });
    if (DataToFilter_Gyro.size() > 2)
    {
        // average accelerations first
        auto N11 = static_cast<double>(DataToFilter_Gyro.size());
        double N12 = 0.0;
        double N22 = 0.0;
        Eigen::VectorXd n1 = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd n2 = Eigen::VectorXd::Zero(6);
        for (const auto& key_val : DataToFilter_Gyro)
        {
            double delta_t = static_cast<double>((key_val.first - imuObsWDelta->insTime).count());
            N12 += delta_t;
            N22 += delta_t * delta_t;
            n1 += key_val.second;
            n2 += delta_t * key_val.second;
        }
        double determinant_inverse = 1.0 / (N11 * N22 - N12 * N12);
        Eigen::VectorXd filtered = determinant_inverse * (N22 * n1 - N12 * n2);
        imuObsWDelta->p_angularRate = filtered.segment(0, 3);
        imuObsWDelta->dtheta = filtered.segment(3, 3);
    }
    return imuObsWDelta;
}

const char* NAV::LowPassFilter::to_string(FilterType value)
{
    switch (value)
    {
    case FilterType::Linear:
        return "Linear fit filter";
    // case FilterType::Experimental:
    //     return "Experimental";
    case FilterType::COUNT:
        return "";
    }
    return "";
}
