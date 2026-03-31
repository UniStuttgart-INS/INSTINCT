// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Skip.hpp"
#include <imgui.h>
#include <limits>

#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::Skip::Skip() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 357, 73 };

    CreateInputPin("Input", Pin::Type::Flow, { NodeData::type() }, &Skip::receiveObs);

    CreateOutputPin("Output", Pin::Type::Flow, { NodeData::type() });
}

NAV::Skip::~Skip()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Skip::typeStatic()
{
    return "Skip";
}

std::string NAV::Skip::type() const
{
    return typeStatic();
}

std::string NAV::Skip::category()
{
    return "Utility";
}

void NAV::Skip::guiConfig()
{
    ImGui::SetNextItemWidth(150.0F);
    if (auto retain = static_cast<int>(_retainEvery);
        ImGui::InputIntL(fmt::format("Retain every Nth message##{}", size_t(id)).c_str(), &retain, 1, std::numeric_limits<int>::max()))
    {
        _retainEvery = static_cast<size_t>(retain);
        flow::ApplyChanges();
    }
}

json NAV::Skip::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "retainEvery", _retainEvery },
    };
}

void NAV::Skip::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("retainEvery")) { j.at("retainEvery").get_to(_retainEvery); }
}

bool NAV::Skip::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _retainCounter = 0;

    return true;
}

void NAV::Skip::afterCreateLink(OutputPin& startPin, InputPin& endPin)
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

void NAV::Skip::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
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

void NAV::Skip::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Check whether timestamp is within the time window
    auto obs = queue.extract_front();

    if ((_retainCounter++ % _retainEvery) == 0)
    {
        _retainCounter = 1;
        invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, obs);
    }
}