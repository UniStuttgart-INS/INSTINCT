#include "Combiner.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/InsObs.hpp"

NAV::Combiner::Combiner()
{
    LOG_TRACE("{}: called", name);

    hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "", Pin::Type::Flow, { InsObs::type() }, &Combiner::receiveData);
    nm::CreateInputPin(this, "", Pin::Type::Flow, { InsObs::type() }, &Combiner::receiveData);
    nm::CreateOutputPin(this, "", Pin::Type::Flow, InsObs::type());
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

bool NAV::Combiner::onCreateLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return true; // Link on Output Port
        }

        // New Link on the Input port, but the previously connected dataIdentifier is different from the new one.
        // Then remove all links.
        if (outputPins.at(OutputPortIndex_Flow).dataIdentifier != startPin->dataIdentifier)
        {
            for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
            {
                nm::DeleteLink(link->id);
            }
        }

        // Update the dataIdentifier of the output pin to the same as input pin
        outputPins.at(OutputPortIndex_Flow).dataIdentifier = startPin->dataIdentifier;

        inputPins.at(InputPortIndex_Flow_First).dataIdentifier = startPin->dataIdentifier;
        inputPins.at(InputPortIndex_Flow_Second).dataIdentifier = startPin->dataIdentifier;

        // Refresh all links connected to the output pin
        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Flow).id))
        {
            nm::RefreshLink(link->id);
        }
    }

    return true;
}

void NAV::Combiner::onDeleteLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return; // Link on Output Port
        }

        for (auto& pin : inputPins)
        {
            if (endPin->id != pin.id && !nm::IsPinLinked(pin.id)) // The other pin is not linked
            {
                for (auto& pinReset : inputPins)
                {
                    pinReset.dataIdentifier = { InsObs::type() };
                }
                break;
            }
        }
    }
}

void NAV::Combiner::receiveData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    if (!(NAV::Node::callbacksEnabled))
    {
        NAV::Node::callbacksEnabled = true;
    }

    invokeCallbacks(OutputPortIndex_Flow, nodeData);
}