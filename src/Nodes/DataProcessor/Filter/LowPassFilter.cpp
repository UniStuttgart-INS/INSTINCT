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

#include <imgui_internal.h>
#include <limits>
#include <set>
#include <type_traits>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), ImuObsWDelta::type(), PosVelAtt::type(), GnssObs::type() };

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::LowPassFilter::LowPassFilter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 812, 530 };

    nm::CreateInputPin(this, "True", Pin::Type::Flow, supportedDataIdentifier, &LowPassFilter::receiveObs);

    nm::CreateOutputPin(this, "Biased", Pin::Type::Flow, supportedDataIdentifier);

    std::mt19937_64 gen(static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist(0, std::numeric_limits<uint64_t>::max() / 2);

    _imuAccelerometerRng.seed = dist(gen);
    _imuGyroscopeRng.seed = dist(gen);

    _imuAccelerometerRWRng.seed = dist(gen);
    _imuGyroscopeRWRng.seed = dist(gen);
    _imuAccelerometerIRWRng.seed = dist(gen);
    _imuGyroscopeIRWRng.seed = dist(gen);

    _positionRng.seed = dist(gen);
    _velocityRng.seed = dist(gen);
    _attitudeRng.seed = dist(gen);
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

    float itemWidth = 470 * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 180 * gui::NodeEditorApplication::windowFontRatio();

    auto inputDoubleWithUnit = [&](const char* title, double& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format) {
        if (auto response = gui::widgets::InputDoubleWithUnit(fmt::format("{}##{}", title, size_t(id)).c_str(), itemWidth, unitWidth,
                                                              &data, unit, combo_items_separated_by_zeros, 0.0, 0.0,
                                                              format, ImGuiInputTextFlags_CharsScientific))
        {
            if (response == gui::widgets::InputWithUnitChange_Input) { LOG_DEBUG("{}: {} changed to {}", nameId(), title, data); }
            if (response == gui::widgets::InputWithUnitChange_Unit) { LOG_DEBUG("{}: {} unit changed to {}", nameId(), title, fmt::underlying(unit)); }
            flow::ApplyChanges();
        }
    };

    auto inputVector3WithUnit = [&](const char* title, Eigen::Vector3d& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format) {
        if (auto response = gui::widgets::InputDouble3WithUnit(fmt::format("{}##{}", title, size_t(id)).c_str(), itemWidth, unitWidth,
                                                               data.data(), unit, combo_items_separated_by_zeros,
                                                               format, ImGuiInputTextFlags_CharsScientific))
        {
            if (response == gui::widgets::InputWithUnitChange_Input) { LOG_DEBUG("{}: {} changed to {}", nameId(), title, data.transpose()); }
            if (response == gui::widgets::InputWithUnitChange_Unit) { LOG_DEBUG("{}: {} unit changed to {}", nameId(), title, fmt::underlying(unit)); }
            flow::ApplyChanges();
        }
    };

    auto rngInput = [&](const char* title, RandomNumberGenerator& rng) {
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##rng.useSeed {} {}", title, size_t(id)).c_str(), &rng.useSeed))
        {
            LOG_DEBUG("{}: {} rng.useSeed changed to {}", nameId(), title, rng.useSeed);
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Use seed?"); }
        ImGui::SameLine();
        if (!rng.useSeed)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX));
        if (ImGui::SliderULong(fmt::format("{} Seed##{}", title, size_t(id)).c_str(), &rng.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu"))
        {
            LOG_DEBUG("{}: {} rng.seed changed to {}", nameId(), title, rng.seed);
            flow::ApplyChanges();
        }
        if (!rng.useSeed)
        {
            ImGui::EndDisabled();
        }
    };

    auto noiseGuiInput = [&]<typename T>(const char* title, T& data, auto& unit, const char* combo_items_separated_by_zeros, const char* format, RandomNumberGenerator& rng) {
        if constexpr (std::is_same_v<T, double>) { inputDoubleWithUnit(title, data, unit, combo_items_separated_by_zeros, format); }
        else if constexpr (std::is_same_v<T, Eigen::Vector3d>) { inputVector3WithUnit(title, data, unit, combo_items_separated_by_zeros, format); }

        rngInput(title, rng);
    };

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta || _inputType == InputType::PosVelAtt)
    {
        ImGui::TextUnformatted("Offsets:");
        ImGui::Indent();
        {
            if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
            {
                inputVector3WithUnit("Accelerometer Bias (platform)", _imuAccelerometerBias_p, _imuAccelerometerBiasUnit, MakeComboItems<Units::ImuAccelerometerUnits>().c_str(), "%.2g");
                inputVector3WithUnit("Gyroscope Bias (platform)", _imuGyroscopeBias_p, _imuGyroscopeBiasUnit, MakeComboItems<Units::ImuGyroscopeUnits>().c_str(), "%.2g");
            }
            else if (_inputType == InputType::PosVelAtt)
            {
                inputVector3WithUnit(fmt::format("Position Bias ({})", _positionBiasUnit == PositionBiasUnits::meter ? "NED" : "LatLonAlt").c_str(),
                                     _positionBias, _positionBiasUnit, "m, m, m\0rad, rad, m\0deg, deg, m\0\0", "%.2g");
                inputVector3WithUnit("Velocity Bias (NED)", _velocityBias, _velocityBiasUnit, "m/s\0\0", "%.2g");
                inputVector3WithUnit("RollPitchYaw Bias", _attitudeBias, _attitudeBiasUnit, "rad\0deg\0\0", "%.2g");
            }
        }
        ImGui::Unindent();
    }

    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta || _inputType == InputType::PosVelAtt)
    {
        ImGui::TextUnformatted("Measurement noise:");
        ImGui::Indent();
        {
            if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
            {
                noiseGuiInput("Accelerometer Noise (Std. dev)", _imuAccelerometerNoise, _imuAccelerometerNoiseUnit, MakeComboItems<Units::ImuAccelerometerNoiseUnits>().c_str(), "%.2g", _imuAccelerometerRng);
                noiseGuiInput("Gyroscope Noise (Std. dev)", _imuGyroscopeNoise, _imuGyroscopeNoiseUnit, MakeComboItems<Units::ImuGyroscopeNoiseUnits>().c_str(), "%.2g", _imuGyroscopeRng);
            }
            else if (_inputType == InputType::PosVelAtt)
            {
                noiseGuiInput(fmt::format("Position Noise ({})", _positionNoiseUnit == PositionNoiseUnits::meter
                                                                         || _positionNoiseUnit == PositionNoiseUnits::rad_rad_m
                                                                         || _positionNoiseUnit == PositionNoiseUnits::deg_deg_m
                                                                     ? "Standard deviation"
                                                                     : "Variance")
                                  .c_str(),
                              _positionNoise, _positionNoiseUnit, "m, m, m\0rad, rad, m\0deg, deg, m\0m^2, m^2, m^2\0rad^2, rad^2, m^2\0deg^2, deg^2, m^2\0\0",
                              "%.2g", _positionRng);
                noiseGuiInput(fmt::format("Velocity Noise ({})", _velocityNoiseUnit == VelocityNoiseUnits::m_s ? "Standard deviation"
                                                                                                               : "Variance")
                                  .c_str(),
                              _velocityNoise, _velocityNoiseUnit, "m/s\0m^2/s^2\0\0", "%.2g", _velocityRng);
                noiseGuiInput(fmt::format("Attitude Noise ({})", _attitudeNoiseUnit == AttitudeNoiseUnits::rad || _attitudeNoiseUnit == AttitudeNoiseUnits::deg
                                                                     ? "Standard deviation"
                                                                     : "Variance")
                                  .c_str(),
                              _attitudeNoise, _attitudeNoiseUnit, "rad\0deg\0rad^2\0deg^2\0\0", "%.2g", _attitudeRng);
            }
        }
        ImGui::Unindent();
    }
    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        ImGui::TextUnformatted("Random walk noise:");
        ImGui::Indent();
        noiseGuiInput("Accelerometer RW (Std. dev)", _imuAccelerometerRW, _imuAccelerometerRWUnit, MakeComboItems<Units::ImuAccelerometerNoiseUnits>().c_str(), "%.2g", _imuAccelerometerRWRng);
        noiseGuiInput("Gyroscope RW (Std. dev)", _imuGyroscopeRW, _imuGyroscopeRWUnit, MakeComboItems<Units::ImuGyroscopeNoiseUnits>().c_str(), "%.2g", _imuGyroscopeRWRng);
        ImGui::Unindent();
        ImGui::TextUnformatted("Integrated Random walk noise:");
        ImGui::Indent();
        noiseGuiInput("Accelerometer IRW (Std. dev)", _imuAccelerometerIRW, _imuAccelerometerIRWUnit, MakeComboItems<Units::ImuAccelerometerIRWUnits>().c_str(), "%.2g", _imuAccelerometerIRWRng);
        noiseGuiInput("Gyroscope IRW (Std. dev)", _imuGyroscopeIRW, _imuGyroscopeIRWUnit, MakeComboItems<Units::ImuGyroscopeIRWUnits>().c_str(), "%.2g", _imuGyroscopeIRWRng);
        ImGui::Unindent();
    }

}

json NAV::LowPassFilter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuAccelerometerBiasUnit"] = _imuAccelerometerBiasUnit;
    j["imuAccelerometerBias_p"] = _imuAccelerometerBias_p;
    j["imuGyroscopeBiasUnit"] = _imuGyroscopeBiasUnit;
    j["imuGyroscopeBias_p"] = _imuGyroscopeBias_p;
    j["imuAccelerometerNoiseUnit"] = _imuAccelerometerNoiseUnit;
    j["imuAccelerometerNoise"] = _imuAccelerometerNoise;
    j["imuAccelerometerRng"] = _imuAccelerometerRng;
    j["imuGyroscopeNoiseUnit"] = _imuGyroscopeNoiseUnit;
    j["imuGyroscopeNoise"] = _imuGyroscopeNoise;
    j["imuGyroscopeRng"] = _imuGyroscopeRng;
    j["imuAccelerometerRWUnit"] = _imuAccelerometerRWUnit;
    j["imuAccelerometerRW"] = _imuAccelerometerRW;
    j["imuAccelerometerRWRng"] = _imuAccelerometerRWRng;

    j["imuGyroscopeRWUnit"] = _imuGyroscopeRWUnit;
    j["imuGyroscopeRW"] = _imuGyroscopeRW;
    j["imuGyroscopeRWRng"] = _imuGyroscopeRWRng;
    j["imuAccelerometerIRWUnit"] = _imuAccelerometerIRWUnit;
    j["imuAccelerometerIRW"] = _imuAccelerometerIRW;
    j["imuAccelerometerIRWRng"] = _imuAccelerometerIRWRng;
    j["imuGyroscopeIRWUnit"] = _imuGyroscopeIRWUnit;
    j["imuGyroscopeIRW"] = _imuGyroscopeIRW;
    j["imuGyroscopeIRWRng"] = _imuGyroscopeIRWRng;

    // #########################################################################################################################################
    j["positionBiasUnit"] = _positionBiasUnit;
    j["positionBias"] = _positionBias;
    j["velocityBiasUnit"] = _velocityBiasUnit;
    j["velocityBias"] = _velocityBias;
    j["attitudeBiasUnit"] = _attitudeBiasUnit;
    j["attitudeBias"] = _attitudeBias;
    j["positionNoiseUnit"] = _positionNoiseUnit;
    j["positionNoise"] = _positionNoise;
    j["positionRng"] = _positionRng;
    j["velocityNoiseUnit"] = _velocityNoiseUnit;
    j["velocityNoise"] = _velocityNoise;
    j["velocityRng"] = _velocityRng;
    j["attitudeNoiseUnit"] = _attitudeNoiseUnit;
    j["attitudeNoise"] = _attitudeNoise;
    j["attitudeRng"] = _attitudeRng;

    return j;
}

void NAV::LowPassFilter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuAccelerometerBiasUnit")) { j.at("imuAccelerometerBiasUnit").get_to(_imuAccelerometerBiasUnit); }
    if (j.contains("imuAccelerometerBias_p")) { j.at("imuAccelerometerBias_p").get_to(_imuAccelerometerBias_p); }
    if (j.contains("imuGyroscopeBiasUnit")) { j.at("imuGyroscopeBiasUnit").get_to(_imuGyroscopeBiasUnit); }
    if (j.contains("imuGyroscopeBias_p")) { j.at("imuGyroscopeBias_p").get_to(_imuGyroscopeBias_p); }
    if (j.contains("imuAccelerometerNoiseUnit")) { j.at("imuAccelerometerNoiseUnit").get_to(_imuAccelerometerNoiseUnit); }
    if (j.contains("imuAccelerometerNoise")) { j.at("imuAccelerometerNoise").get_to(_imuAccelerometerNoise); }
    if (j.contains("imuAccelerometerRng")) { j.at("imuAccelerometerRng").get_to(_imuAccelerometerRng); }
    if (j.contains("imuGyroscopeNoiseUnit")) { j.at("imuGyroscopeNoiseUnit").get_to(_imuGyroscopeNoiseUnit); }
    if (j.contains("imuGyroscopeNoise")) { j.at("imuGyroscopeNoise").get_to(_imuGyroscopeNoise); }
    if (j.contains("imuGyroscopeRng")) { j.at("imuGyroscopeRng").get_to(_imuGyroscopeRng); }

    if (j.contains("imuAccelerometerRWUnit")) { j.at("imuAccelerometerRWUnit").get_to(_imuAccelerometerRWUnit); }
    if (j.contains("imuAccelerometerRW")) { j.at("imuAccelerometerRW").get_to(_imuAccelerometerRW); }
    if (j.contains("imuAccelerometerRWRng")) { j.at("imuAccelerometerRWRng").get_to(_imuAccelerometerRWRng); }
    if (j.contains("imuGyroscopeRWUnit")) { j.at("imuGyroscopeRWUnit").get_to(_imuGyroscopeRWUnit); }
    if (j.contains("imuGyroscopeRW")) { j.at("imuGyroscopeRW").get_to(_imuGyroscopeRW); }
    if (j.contains("imuGyroscopeRWRng")) { j.at("imuGyroscopeRWRng").get_to(_imuGyroscopeRWRng); }
    if (j.contains("imuAccelerometerIRWUnit")) { j.at("imuAccelerometerIRWUnit").get_to(_imuAccelerometerIRWUnit); }
    if (j.contains("imuAccelerometerIRW")) { j.at("imuAccelerometerIRW").get_to(_imuAccelerometerIRW); }
    if (j.contains("imuAccelerometerIRWRng")) { j.at("imuAccelerometerIRWRng").get_to(_imuAccelerometerIRWRng); }
    if (j.contains("imuGyroscopeIRWUnit")) { j.at("imuGyroscopeIRWUnit").get_to(_imuGyroscopeIRWUnit); }
    if (j.contains("imuGyroscopeIRW")) { j.at("imuGyroscopeIRW").get_to(_imuGyroscopeIRW); }
    if (j.contains("imuGyroscopeIRWRng")) { j.at("imuGyroscopeIRWRng").get_to(_imuGyroscopeIRWRng); }
    // #########################################################################################################################################
    if (j.contains("positionBiasUnit")) { j.at("positionBiasUnit").get_to(_positionBiasUnit); }
    if (j.contains("positionBias")) { j.at("positionBias").get_to(_positionBias); }
    if (j.contains("velocityBiasUnit")) { j.at("velocityBiasUnit").get_to(_velocityBiasUnit); }
    if (j.contains("velocityBias")) { j.at("velocityBias").get_to(_velocityBias); }
    if (j.contains("attitudeBiasUnit")) { j.at("attitudeBiasUnit").get_to(_attitudeBiasUnit); }
    if (j.contains("attitudeBias")) { j.at("attitudeBias").get_to(_attitudeBias); }
    if (j.contains("positionNoiseUnit")) { j.at("positionNoiseUnit").get_to(_positionNoiseUnit); }
    if (j.contains("positionNoise")) { j.at("positionNoise").get_to(_positionNoise); }
    if (j.contains("positionRng")) { j.at("positionRng").get_to(_positionRng); }
    if (j.contains("velocityNoiseUnit")) { j.at("velocityNoiseUnit").get_to(_velocityNoiseUnit); }
    if (j.contains("velocityNoise")) { j.at("velocityNoise").get_to(_velocityNoise); }
    if (j.contains("velocityRng")) { j.at("velocityRng").get_to(_velocityRng); }
    if (j.contains("attitudeNoiseUnit")) { j.at("attitudeNoiseUnit").get_to(_attitudeNoiseUnit); }
    if (j.contains("attitudeNoise")) { j.at("attitudeNoise").get_to(_attitudeNoise); }
    if (j.contains("attitudeRng")) { j.at("attitudeRng").get_to(_attitudeRng); }
}

bool NAV::LowPassFilter::resetNode()
{
    LOG_TRACE("{}: called", nameId());


    if (_inputType == InputType::ImuObs || _inputType == InputType::ImuObsWDelta)
    {
        
        DataToFilter_Accel.clear();
        DataToFilter_Gyro.clear();        

    }
    else if (_inputType == InputType::PosVelAtt)
    {
        _positionRng.resetSeed(size_t(id));
        _velocityRng.resetSeed(size_t(id));
        _attitudeRng.resetSeed(size_t(id));
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
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { PosVelAtt::type() }))
    {
        _inputType = InputType::PosVelAtt;
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
    else if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier, { PosVelAtt::type() }))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(obs))));
    }
}

std::shared_ptr<NAV::ImuObs> NAV::LowPassFilter::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
   
   
         // first we filter accelerations
         DataToFilter_Accel[imuObs->insTime] = imuObs->p_acceleration;
        
         double test_dt_accel = 0.1;
         // remove all entries that are outside filter time window
         std::erase_if(DataToFilter_Accel, [&](const auto& pair){ return static_cast<double>((imuObs->insTime-pair.first).count()) > test_dt_accel ; });
        
        
        
        if (DataToFilter_Accel.size()>2)
        {
           //average accelerations first
           double N11 = static_cast<double>(DataToFilter_Accel.size());
           double N12 = 0.0;
           double N22 = 0.0;
           Eigen::VectorXd n1 =  Eigen::VectorXd::Zero(3);
           Eigen::VectorXd n2 =  Eigen::VectorXd::Zero(3);
           for (const auto& key_val : DataToFilter_Accel)
           {
               double delta_t = static_cast<double>((key_val.first -  imuObs->insTime).count());
               N12+=delta_t;
               N22+=delta_t*delta_t;
               n1+=key_val.second;
               n2+=delta_t*key_val.second;
           } 
           double determinant_inverse =  1.0 / (N11*N22-N12*N12);
           Eigen::VectorXd filtered = determinant_inverse * (N22*n1 -N12*n2);
           imuObs->p_acceleration =  filtered;
        }
        
        // then we filter gyro data
        DataToFilter_Gyro[imuObs->insTime] = imuObs->p_angularRate;
       
        double test_dt_gyro = 0.1;
       
          // remove all entries that are outside filter time window
        std::erase_if(DataToFilter_Gyro, [&](const auto& pair){ return static_cast<double>((imuObs->insTime-pair.first).count()) > test_dt_gyro ; });
        if (DataToFilter_Gyro.size()>2)
        {
           
           //average accelerations first
           double N11 = static_cast<double>(DataToFilter_Gyro.size());
           double N12 = 0.0;
           double N22 = 0.0;
           Eigen::VectorXd n1 =  Eigen::VectorXd::Zero(3);
           Eigen::VectorXd n2 =  Eigen::VectorXd::Zero(3);
           for (const auto& key_val : DataToFilter_Gyro)
           {
               double delta_t = static_cast<double>((key_val.first -  imuObs->insTime).count());
               N12+=delta_t;
               N22+=delta_t*delta_t;
               n1+=key_val.second;
               n2+=delta_t*key_val.second;
           }   
           double determinant_inverse =  1.0 / (N11*N22-N12*N12);
           Eigen::VectorXd filtered = determinant_inverse * (N22*n1 -N12*n2);
           imuObs->p_angularRate =  filtered;
        }  
        

    return imuObs;
}

std::shared_ptr<NAV::ImuObsWDelta> NAV::LowPassFilter::receiveImuObsWDelta(const std::shared_ptr<ImuObsWDelta>& imuObsWDelta)
{
    receiveImuObs(imuObsWDelta);

    return imuObsWDelta;
}

std::shared_ptr<NAV::PosVelAtt> NAV::LowPassFilter::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    // Position Bias in latLonAlt in [rad, rad, m]
    Eigen::Vector3d lla_positionBias = Eigen::Vector3d::Zero();
    switch (_positionBiasUnit)
    {
    case PositionBiasUnits::meter:
    {
        Eigen::Vector3d e_positionBias = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionBias;
        if (!e_positionBias.isZero())
        {
            lla_positionBias = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionBias) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionBiasUnits::rad_rad_m:
        lla_positionBias = _positionBias;
        break;
    case PositionBiasUnits::deg_deg_m:
        lla_positionBias = Eigen::Vector3d{ deg2rad(_positionBias(0)), deg2rad(_positionBias(1)), _positionBias(2) };
        break;
    }
    LOG_DATA("{}: lla_positionBias = {} [rad, rad, m]", nameId(), lla_positionBias.transpose());

    // Velocity bias in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityBias = Eigen::Vector3d::Zero();
    switch (_velocityBiasUnit)
    {
    case VelocityBiasUnits::m_s:
        n_velocityBias = _velocityBias;
        break;
    }
    LOG_DATA("{}: n_velocityBias = {} [m/s]", nameId(), n_velocityBias.transpose());

    // Roll, pitch, yaw bias in [rad]
    Eigen::Vector3d attitudeBias = Eigen::Vector3d::Zero();
    switch (_attitudeBiasUnit)
    {
    case AttitudeBiasUnits::rad:
        attitudeBias = _attitudeBias;
        break;
    case AttitudeBiasUnits::deg:
        attitudeBias = deg2rad(_attitudeBias);
        break;
    }
    LOG_DATA("{}: attitudeBias = {} [rad]", nameId(), attitudeBias.transpose());

    // #########################################################################################################################################

    // Position Noise standard deviation in latitude, longitude and altitude [rad, rad, m]
    Eigen::Vector3d lla_positionNoiseStd = Eigen::Vector3d::Zero();
    switch (_positionNoiseUnit)
    {
    case PositionNoiseUnits::meter:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise;
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad_rad_m:
        lla_positionNoiseStd = _positionNoise;
        break;
    case PositionNoiseUnits::deg_deg_m:
        lla_positionNoiseStd = deg2rad(_positionNoise);
        break;
    case PositionNoiseUnits::meter2:
    {
        Eigen::Vector3d e_positionNoiseStd = trafo::e_Quat_n(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise.cwiseSqrt();
        if (!e_positionNoiseStd.isZero())
        {
            lla_positionNoiseStd = trafo::ecef2lla_WGS84(posVelAtt->e_position() + e_positionNoiseStd) - posVelAtt->lla_position();
        }
        break;
    }
    case PositionNoiseUnits::rad2_rad2_m2:
        lla_positionNoiseStd = _positionNoise.cwiseSqrt();
        break;
    case PositionNoiseUnits::deg2_deg2_m2:
        lla_positionNoiseStd = deg2rad(_positionNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: lla_positionNoiseStd = {} [rad, rad, m]", nameId(), lla_positionNoiseStd.transpose());

    // Velocity Noise standard deviation in local-navigation coordinates in [m/s]
    Eigen::Vector3d n_velocityNoiseStd = Eigen::Vector3d::Zero();
    switch (_velocityNoiseUnit)
    {
    case VelocityNoiseUnits::m_s:
        n_velocityNoiseStd = _velocityNoise;
        break;
    case VelocityNoiseUnits::m2_s2:
        n_velocityNoiseStd = _velocityNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: n_velocityNoiseStd = {} [m/s]", nameId(), n_velocityNoiseStd.transpose());

    // Attitude Noise standard deviation in [rad]
    Eigen::Vector3d attitudeNoiseStd = Eigen::Vector3d::Zero();
    switch (_attitudeNoiseUnit)
    {
    case AttitudeNoiseUnits::rad:
        attitudeNoiseStd = _attitudeNoise;
        break;
    case AttitudeNoiseUnits::deg:
        attitudeNoiseStd = deg2rad(_attitudeNoise);
        break;
    case AttitudeNoiseUnits::rad2:
        attitudeNoiseStd = _attitudeNoise.cwiseSqrt();
        break;
    case AttitudeNoiseUnits::deg2:
        attitudeNoiseStd = deg2rad(_attitudeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: attitudeNoiseStd = {} [rad]", nameId(), attitudeNoiseStd.transpose());

    // #########################################################################################################################################

    posVelAtt->setState_n(posVelAtt->lla_position()
                              + lla_positionBias
                              + Eigen::Vector3d{ _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(0)),
                                                 _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(1)),
                                                 _positionRng.getRand_normalDist(0.0, lla_positionNoiseStd(2)) },
                          posVelAtt->n_velocity()
                              + n_velocityBias
                              + Eigen::Vector3d{ _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(0)),
                                                 _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(1)),
                                                 _velocityRng.getRand_normalDist(0.0, n_velocityNoiseStd(2)) },
                          trafo::n_Quat_b(posVelAtt->rollPitchYaw()
                                          + attitudeBias
                                          + Eigen::Vector3d{ _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(0)),
                                                             _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(1)),
                                                             _attitudeRng.getRand_normalDist(0.0, attitudeNoiseStd(2)) }));

    return posVelAtt;
}

