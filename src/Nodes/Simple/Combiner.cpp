#include "Combiner.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeRegistry.hpp"

#include "NodeData/InsObs.hpp"

NAV::Combiner::Combiner()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "", Pin::Type::Flow, { InsObs::type() }, &Combiner::receiveData);
    nm::CreateInputPin(this, "", Pin::Type::Flow, { InsObs::type() }, &Combiner::receiveData);
    nm::CreateOutputPin(this, "", Pin::Type::Flow, { InsObs::type() });
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
    return "Simple";
}

void NAV::Combiner::setPinIdentifiers(size_t connectedPinIndex, size_t otherPinIndex, const std::vector<std::string>& dataIdentifiers)
{
    LOG_DEBUG("{}: Setting DataIdentifier on pinIndex {}", nameId(), otherPinIndex);

    inputPins.at(connectedPinIndex).dataIdentifier = dataIdentifiers;

    if (!nm::IsPinLinked(inputPins.at(otherPinIndex)))
    {
        inputPins.at(otherPinIndex).dataIdentifier = dataIdentifiers;
        for (const auto& dataIdentifier : dataIdentifiers)
        {
            auto parentIdentifiers = NodeRegistry::GetParentNodeDataTypes(dataIdentifier);
            std::erase(parentIdentifiers, InsObs::type());
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
            std::erase(parentIdentifiers, InsObs::type());
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
    for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_FLOW)))
    {
        auto* startPin = nm::FindOutputPin(link->startPinId);
        auto* endPin = nm::FindInputPin(link->endPinId);
        if (startPin && endPin)
        {
            if (startPin->canCreateLink(*endPin))
            {
                continue;
            }
        }

        nm::DeleteLink(link->id);
    }

    // Refresh all links connected to the output pin if the type changed
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != oldDataIdentifiers)
    {
        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_FLOW)))
        {
            nm::RefreshLink(link->id);
        }
    }
}

bool NAV::Combiner::onCreateLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id) // Link on Output Port
    {
        if (!nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_FLOW_FIRST))
            && !nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_FLOW_SECOND)))
        {
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = endPin.dataIdentifier;
        }
        return true;
    }

    size_t connectedPinIndex = pinIndexFromId(endPin.id);
    size_t otherPinIndex = connectedPinIndex == INPUT_PORT_INDEX_FLOW_FIRST ? INPUT_PORT_INDEX_FLOW_SECOND : INPUT_PORT_INDEX_FLOW_FIRST;

    auto outputPinIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;

    setPinIdentifiers(connectedPinIndex, otherPinIndex, startPin.dataIdentifier);

    updateOutputPin(outputPinIdentifier);

    return true;
}

void NAV::Combiner::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (endPin.parentNode->id != id) // Link on Output Pin
    {
        return;
    }

    // Link on Input Pin
    for (auto& pin : inputPins)
    {
        if (endPin.id != pin.id) // The other pin (which is not deleted)
        {
            if (nm::IsPinLinked(pin))
            {
                size_t connectedPinIndex = pinIndexFromId(pin.id);
                size_t otherPinIndex = connectedPinIndex == INPUT_PORT_INDEX_FLOW_FIRST ? INPUT_PORT_INDEX_FLOW_SECOND : INPUT_PORT_INDEX_FLOW_FIRST;

                Pin* otherNodePin = nm::FindConnectedPinToInputPin(pin);

                auto outputPinIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;

                setPinIdentifiers(connectedPinIndex, otherPinIndex, otherNodePin->dataIdentifier);

                updateOutputPin(outputPinIdentifier);
            }
            else
            {
                for (auto& pinReset : inputPins)
                {
                    pinReset.dataIdentifier = { InsObs::type() };
                }
            }
            break;
        }
    }
}

void NAV::Combiner::receiveData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (!(NAV::Node::callbacksEnabled))
    {
        NAV::Node::callbacksEnabled = true;
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, nodeData);
}