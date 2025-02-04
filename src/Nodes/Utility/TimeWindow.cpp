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
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::TimeWindow::TimeWindow() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 500, 290 };

    nm::CreateInputPin(this, "Input", Pin::Type::Flow, { NodeData::type() }, &TimeWindow::receiveObs);

    nm::CreateOutputPin(this, "Output", Pin::Type::Flow, { NodeData::type() });
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
    return "Utility";
}

void NAV::TimeWindow::guiConfig()
{
    if (ImGui::Checkbox(fmt::format("Inverse Window##{}", size_t(id)).c_str(), &_inverseWindow))
    {
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("Normal: [startTime, endTime]\n"
                             "Inverse: (-∞, startTime), (endTime, ∞)");

    if (ImGui::BeginTable(fmt::format("Time Window##{}", size_t(id)).c_str(), 2, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Beginning");
        ImGui::TableSetupColumn("End");
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < _startEndTime.size(); i++)
        {
            ImGui::TableNextColumn();
            if (gui::widgets::TimeEdit(fmt::format("Time edit##{} {}", i, size_t(id)).c_str(), _startEndTime.at(i), _timeEditFormat.at(i)))
            {
                LOG_DEBUG("{}: {}Time = {}", nameId(), i == 0 ? "start" : "end", _startEndTime.at(i));
                flow::ApplyChanges();
                if (_startEndTime[0] >= _startEndTime[1]) { doDeinitialize(); }
            }
        }

        ImGui::EndTable();
    }
}

json NAV::TimeWindow::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["startEndTime"] = _startEndTime;
    j["timeEditFormat"] = _timeEditFormat;
    j["inverseWindow"] = _inverseWindow;

    return j;
}

void NAV::TimeWindow::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("startEndTime"))
    {
        j.at("startEndTime").get_to(_startEndTime);
    }
    if (j.contains("timeEditFormat"))
    {
        j.at("timeEditFormat").get_to(_timeEditFormat);
    }
    if (j.contains("inverseWindow"))
    {
        j.at("inverseWindow").get_to(_inverseWindow);
    }
}

bool NAV::TimeWindow::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (_startEndTime[0] >= _startEndTime[1])
    {
        LOG_ERROR("{}: startTime >= endTime is not allowed. Please reconfigure the node.", nameId());
        return false;
    }

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
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = { NodeData::type() };
    }
}

void NAV::TimeWindow::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Check whether timestamp is within the time window
    auto obs = queue.extract_front();
    if (_inverseWindow)
    {
        if (obs->insTime < _startEndTime[0] || obs->insTime > _startEndTime[1])
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, obs);
        }
    }
    else
    {
        if (obs->insTime >= _startEndTime[0] && obs->insTime <= _startEndTime[1])
        {
            invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, obs);
        }
        else if (obs->insTime > _startEndTime[1])
        {
            inputPins.at(INPUT_PORT_INDEX_FLOW).queueBlocked = true;
            inputPins.at(INPUT_PORT_INDEX_FLOW).queue.clear();
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).noMoreDataAvailable = true;
        }
    }
}