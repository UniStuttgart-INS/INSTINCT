// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeWindow.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include <imgui_internal.h>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), PosVelAtt::type() };
} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::TimeWindow::TimeWindow() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 858, 975 }; // FIXME: default window size

    nm::CreateInputPin(this, "Input", Pin::Type::Flow, supportedDataIdentifier, &TimeWindow::receiveObs);

    nm::CreateOutputPin(this, "Output", Pin::Type::Flow, supportedDataIdentifier);
}

NAV::TimeWindow::~TimeWindow()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TimeWindow::typeStatic()
{
    return "TimeWindow";
}

std::string NAV::TimeWindow::type() const
{
    return typeStatic();
}

std::string NAV::TimeWindow::category()
{
    return "DataProcessor";
}

void NAV::TimeWindow::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    float configWidth = 75.0F;

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Time format##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_timeFormat), "MJD\0JD\0GPST\0UTC\0\0"))
    {
        LOG_DEBUG("{}: Time format changed to {}", nameId(), _timeFormat);
        flow::ApplyChanges();
    }

    ImGui::Separator();

    ImGui::TextUnformatted("Beginning of time-window:");

    if (_timeFormat == TimeFormats::MJD || _timeFormat == TimeFormats::JD)
    {
        if (ImGui::InputDoubleL(fmt::format("Full days of the Modified Julien Date [UTC]##{}", size_t(id)).c_str(), &_days, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
        {
            LOG_DEBUG("{}: days changed to {}", nameId(), _days);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("The inverse of this rate is used as the initial 'dt' for the Kalman Filter Prediction (Phi and Q).");

        if (ImGui::InputDoubleL(fmt::format("Decimal fractions of a day of the Modified Julien Date [UTC]##{}", size_t(id)).c_str(), &_decFrac, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _decFrac);
            flow::ApplyChanges();
        }
    }
    else if (_timeFormat == TimeFormats::GPST)
    {
        if (ImGui::InputDoubleL(fmt::format("gpsCycle##{}", size_t(id)).c_str(), &_gpsCycle, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
        {
            LOG_DEBUG("{}: days changed to {}", nameId(), _gpsCycle);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("The inverse of this rate is used as the initial 'dt' for the Kalman Filter Prediction (Phi and Q).");

        if (ImGui::InputDoubleL(fmt::format("gpsWeek##{}", size_t(id)).c_str(), &_gpsWeek, 1e-3, 1e4, 0.0, 0.0, "%.0f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _gpsWeek);
            flow::ApplyChanges();
        }

        if (ImGui::InputDoubleL(fmt::format("tow##{}", size_t(id)).c_str(), &_gpsTow, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _gpsTow);
            flow::ApplyChanges();
        }
    }
    else if (_timeFormat == TimeFormats::UTC)
    {
        if (ImGui::InputDoubleL(fmt::format("year##{}", size_t(id)).c_str(), &_year, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _year);
            flow::ApplyChanges();
        }
        if (ImGui::InputDoubleL(fmt::format("month##{}", size_t(id)).c_str(), &_month, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _month);
            flow::ApplyChanges();
        }
        if (ImGui::InputDoubleL(fmt::format("day##{}", size_t(id)).c_str(), &_day, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _day);
            flow::ApplyChanges();
        }
        if (ImGui::InputDoubleL(fmt::format("hour##{}", size_t(id)).c_str(), &_hour, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _hour);
            flow::ApplyChanges();
        }
        if (ImGui::InputDoubleL(fmt::format("min##{}", size_t(id)).c_str(), &_min, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _min);
            flow::ApplyChanges();
        }
        if (ImGui::InputDoubleL(fmt::format("sec##{}", size_t(id)).c_str(), &_sec, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction changed to {}", nameId(), _sec);
            flow::ApplyChanges();
        }
    }
    else
    {
        LOG_ERROR("{}: No time format specified", nameId());
    }

    ImGui::Separator();

    ImGui::TextUnformatted("End of time-window:");
}

json NAV::TimeWindow::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return j;
}

void NAV::TimeWindow::restore(json const& j)
{
    if (j) // FIXME: dummy
    {}
    LOG_TRACE("{}: called", nameId());
}

bool NAV::TimeWindow::resetNode()
{
    // TODO: reset
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::TimeWindow::afterCreateLink(OutputPin& startPin, InputPin& endPin)
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

void NAV::TimeWindow::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
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

void NAV::TimeWindow::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Select the correct data type and make a copy of the node data to modify
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(queue.extract_front())));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(queue.extract_front())));
    }
}

void NAV::TimeWindow::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    if (imuObs) // FIXME: dummy
    {}
}

void NAV::TimeWindow::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    if (posVelAtt) // FIXME: dummy
    {}
}