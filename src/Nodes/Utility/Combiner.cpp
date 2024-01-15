// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Combiner.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeRegistry.hpp"

#include "NodeData/NodeData.hpp"

NAV::Combiner::Combiner()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "", Pin::Type::Flow, { NodeData::type() }, &Combiner::receiveData);
    nm::CreateInputPin(this, "", Pin::Type::Flow, { NodeData::type() }, &Combiner::receiveData);
    nm::CreateOutputPin(this, "", Pin::Type::Flow, { NodeData::type() });
}

NAV::Combiner::~Combiner()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Combiner::typeStatic()
{
    return "Combiner";
}

std::string NAV::Combiner::type() const
{
    return typeStatic();
}

std::string NAV::Combiner::category()
{
    return "Utility";
}

void NAV::Combiner::setPinIdentifiers(size_t connectedPinIndex, size_t otherPinIndex, const std::vector<std::string>& dataIdentifiers)
{
    LOG_DEBUG("{}: Setting DataIdentifier on pinIndex {}", nameId(), otherPinIndex);

    inputPins.at(connectedPinIndex).dataIdentifier = dataIdentifiers;

    if (!inputPins.at(otherPinIndex).isPinLinked())
    {
        inputPins.at(otherPinIndex).dataIdentifier = dataIdentifiers;
        for (const auto& dataIdentifier : dataIdentifiers)
        {
            auto parentIdentifiers = NodeRegistry::GetParentNodeDataTypes(dataIdentifier);
            std::erase(parentIdentifiers, NodeData::type());
            inputPins.at(otherPinIndex).dataIdentifier.insert(inputPins.at(otherPinIndex).dataIdentifier.end(), parentIdentifiers.rbegin(), parentIdentifiers.rend());
        }

        // Update the dataIdentifier of the output pin to the same as input pin
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = dataIdentifiers;
    }
    else
    {
        std::vector<std::string> combinedIdentifiers;
        for (const auto& dataIdentifier : inputPins.at(connectedPinIndex).dataIdentifier)
        {
            if (auto iter = std::find(inputPins.at(otherPinIndex).dataIdentifier.begin(), inputPins.at(otherPinIndex).dataIdentifier.end(), dataIdentifier);
                iter != inputPins.at(otherPinIndex).dataIdentifier.end())
            {
                combinedIdentifiers.push_back(*iter);
            }
        }
        if (!combinedIdentifiers.empty())
        {
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = combinedIdentifiers;
            return;
        }

        std::vector<std::string> connectedPinParents;
        for (const auto& dataIdentifier : inputPins.at(connectedPinIndex).dataIdentifier)
        {
            auto parentIdentifiers = NodeRegistry::GetParentNodeDataTypes(dataIdentifier);
            std::erase(parentIdentifiers, NodeData::type());
            connectedPinParents.insert(connectedPinParents.begin(), parentIdentifiers.begin(), parentIdentifiers.end());
        }

        for (const auto& dataIdentifier : inputPins.at(otherPinIndex).dataIdentifier)
        {
            if (std::find(connectedPinParents.begin(), connectedPinParents.end(), dataIdentifier) != connectedPinParents.end())
            {
                outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = inputPins.at(otherPinIndex).dataIdentifier;
            }
            else
            {
                outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = inputPins.at(connectedPinIndex).dataIdentifier;
            }
        }
    }
}

void NAV::Combiner::updateOutputPin(const std::vector<std::string>& oldDataIdentifiers)
{
    // Check if connected links on output port are still valid
    for (auto& link : outputPins.at(OUTPUT_PORT_INDEX_FLOW).links)
    {
        if (auto* endPin = link.getConnectedPin())
        {
            if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).canCreateLink(*endPin))
            {
                continue;
            }

            // If the link is not valid anymore, delete it
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).deleteLink(*endPin);
        }
    }

    // Refresh all links connected to the output pin if the type changed
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != oldDataIdentifiers)
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

bool NAV::Combiner::onCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id) // Link on Output Port
    {
        if (!inputPins.at(INPUT_PORT_INDEX_FLOW_FIRST).isPinLinked()
            && !inputPins.at(INPUT_PORT_INDEX_FLOW_SECOND).isPinLinked())
        {
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = endPin.dataIdentifier;
        }
        return true;
    }

    size_t connectedPinIndex = inputPinIndexFromId(endPin.id);
    size_t otherPinIndex = connectedPinIndex == INPUT_PORT_INDEX_FLOW_FIRST ? INPUT_PORT_INDEX_FLOW_SECOND : INPUT_PORT_INDEX_FLOW_FIRST;

    auto outputPinIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;

    setPinIdentifiers(connectedPinIndex, otherPinIndex, startPin.dataIdentifier);

    updateOutputPin(outputPinIdentifier);

    return true;
}

void NAV::Combiner::afterDeleteLink([[maybe_unused]] OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id) // Link on Output Pin
    {
        return;
    }

    // Link on Input Pin
    for (size_t pinIdx = 0; pinIdx < inputPins.size(); pinIdx++)
    {
        auto& pin = inputPins[pinIdx];
        if (endPin.id != pin.id) // The other pin (which is not deleted)
        {
            if (pin.isPinLinked())
            {
                size_t otherPinIndex = pinIdx == INPUT_PORT_INDEX_FLOW_FIRST ? INPUT_PORT_INDEX_FLOW_SECOND : INPUT_PORT_INDEX_FLOW_FIRST;

                auto* otherNodePin = pin.link.getConnectedPin();

                auto outputPinIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;

                setPinIdentifiers(pinIdx, otherPinIndex, otherNodePin->dataIdentifier);

                updateOutputPin(outputPinIdentifier);
            }
            else
            {
                for (auto& pinReset : inputPins)
                {
                    pinReset.dataIdentifier = { NodeData::type() };
                }
            }
            break;
        }
    }
}

void NAV::Combiner::receiveData(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    if (!(NAV::Node::callbacksEnabled))
    {
        NAV::Node::callbacksEnabled = true;
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, queue.extract_front());
}