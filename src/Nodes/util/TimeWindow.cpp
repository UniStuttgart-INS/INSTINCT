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
    _guiConfigDefaultWindowSize = { 320, 450 }; // FIXME: default window size

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
    float configWidth = 100.0F;

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Time format##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_timeFormat), "MJD\0JD\0GPST\0YMDHMS\0\0"))
    {
        LOG_DEBUG("{}: Time format changed to {}", nameId(), _timeFormat);
        flow::ApplyChanges();
    }

    ImGui::Separator();

    ImGui::TextUnformatted("Beginning of time-window:");

    if (_timeFormat == TimeFormats::MJD || _timeFormat == TimeFormats::JD)
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Days##{}", size_t(id)).c_str(), &_days, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: days = {}", nameId(), _days);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputDoubleL(fmt::format("Decimal fraction of a day##{}", size_t(id)).c_str(), &_decFrac, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: decimal fraction = {}", nameId(), _decFrac);
            flow::ApplyChanges();
        }
        if (_timeFormat == TimeFormats::MJD)
        {
            _mjdStart = InsTime_MJD{ _days, _decFrac };
        }
        if (_timeFormat == TimeFormats::JD)
        {
            _jdStart = InsTime_JD{ _days, _decFrac };
        }
    }
    else if (_timeFormat == TimeFormats::GPST)
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("GPS Cycle##{}", size_t(id)).c_str(), &_gpsCycle, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: GPS cycle = {}", nameId(), _gpsCycle);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("GPS Week##{}", size_t(id)).c_str(), &_gpsWeek, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: GPS week = {}", nameId(), _gpsWeek);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputDoubleL(fmt::format("GPS ToW##{}", size_t(id)).c_str(), &_gpsTow, 1e-3, 1e4, 0.0, 0.0, "%f"))
        {
            LOG_DEBUG("{}: GPS ToW = {}", nameId(), _gpsTow);
            flow::ApplyChanges();
        }

        _gpsWeekTowStart = InsTime_GPSweekTow{ _gpsCycle, _gpsWeek, _gpsTow };
    }
    else if (_timeFormat == TimeFormats::YMDHMS)
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Year##{}", size_t(id)).c_str(), &_year, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: year = {}", nameId(), _year);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Month##{}", size_t(id)).c_str(), &_month, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: month = {}", nameId(), _month);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Day##{}", size_t(id)).c_str(), &_day, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: day = {}", nameId(), _day);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Hour##{}", size_t(id)).c_str(), &_hour, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: hour = {}", nameId(), _hour);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputInt(fmt::format("Minute##{}", size_t(id)).c_str(), &_min, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: minute = {}", nameId(), _min);
            flow::ApplyChanges();
        }
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputDoubleL(fmt::format("Second##{}", size_t(id)).c_str(), &_sec, 0, InsTimeUtil::SECONDS_PER_MINUTE - 1, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
        {
            LOG_DEBUG("{}: second = {}", nameId(), _sec);
            flow::ApplyChanges();
        }

        _ymdhmsStart = InsTime_YMDHMS{ _year, _month, _day, _hour, _min, _sec };
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