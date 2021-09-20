#include "Combiner.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeRegistry.hpp"

#include "NodeData/InsObs.hpp"

NAV::Combiner::Combiner()
{
    LOG_TRACE("{}: called", name);

    hasConfig = false;
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

    if (!nm::IsPinLinked(inputPins.at(otherPinIndex).id))
    {
        inputPins.at(otherPinIndex).dataIdentifier = dataIdentifiers;
        for (const auto& dataIdentifier : dataIdentifiers)
        {
            auto parentIdentifiers = NodeRegistry::GetParentNodeDataTypes(dataIdentifier);
            std::erase(parentIdentifiers, InsObs::type());
            inputPins.at(otherPinIndex).dataIdentifier.insert(inputPins.at(otherPinIndex).dataIdentifier.begin(), parentIdentifiers.begin(), parentIdentifiers.end());
        }

        // Update the dataIdentifier of the output pin to the same as input pin
        outputPins.at(OutputPortIndex_Flow).dataIdentifier = dataIdentifiers;
    }
    else
    {
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
                outputPins.at(OutputPortIndex_Flow).dataIdentifier = inputPins.at(otherPinIndex).dataIdentifier;
            }
            else
            {
                outputPins.at(OutputPortIndex_Flow).dataIdentifier = inputPins.at(connectedPinIndex).dataIdentifier;
            }
        }
    }
}

void NAV::Combiner::updateOutputPin(const std::vector<std::string>& oldDataIdentifiers)
{
    // Check if connected links on output port are still valid
    for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
    {
        auto* startPin = nm::FindPin(link->startPinId);
        auto* endPin = nm::FindPin(link->endPinId);
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
    if (outputPins.at(OutputPortIndex_Flow).dataIdentifier != oldDataIdentifiers)
    {
        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
        {
            nm::RefreshLink(link->id);
        }
    }
}

bool NAV::Combiner::onCreateLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return true; // Link on Output Port
        }

        size_t connectedPinIndex = pinIndexFromId(endPin->id);
        size_t otherPinIndex = connectedPinIndex == InputPortIndex_Flow_First ? InputPortIndex_Flow_Second : InputPortIndex_Flow_First;

        auto outputPinIdentifier = outputPins.at(OutputPortIndex_Flow).dataIdentifier;

        setPinIdentifiers(connectedPinIndex, otherPinIndex, startPin->dataIdentifier);

        updateOutputPin(outputPinIdentifier);
    }

    return true;
}

void NAV::Combiner::afterDeleteLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return; // Link on Output Pin
        }

        // Link on Input Pin
        for (auto& pin : inputPins)
        {
            if (endPin->id != pin.id) // The other pin (which is not deleted)
            {
                if (nm::IsPinLinked(pin.id))
                {
                    size_t connectedPinIndex = pinIndexFromId(pin.id);
                    size_t otherPinIndex = connectedPinIndex == InputPortIndex_Flow_First ? InputPortIndex_Flow_Second : InputPortIndex_Flow_First;

                    Pin* otherNodePin = nm::FindConnectedPinToInputPin(pin.id);

                    auto outputPinIdentifier = outputPins.at(OutputPortIndex_Flow).dataIdentifier;

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
}

void NAV::Combiner::receiveData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (!(NAV::Node::callbacksEnabled))
    {
        NAV::Node::callbacksEnabled = true;
    }

    invokeCallbacks(OutputPortIndex_Flow, nodeData);
}