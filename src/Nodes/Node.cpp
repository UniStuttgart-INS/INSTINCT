#include "Node.hpp"

#include <stdexcept>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

void NAV::Node::guiConfig() {}

json NAV::Node::save() const { return {}; }

void NAV::Node::restore(const json& /*j*/) {}

void NAV::Node::restoreAtferLink(const json& /*j*/) {}

bool NAV::Node::initializeNode()
{
    if (!enabled)
    {
        return false;
    }

    // Lock the node against recursive calling
    isInitializing_ = true;

    if (isInitialized())
    {
        resetNode();
        deinitializeNode();
    }

    LOG_DEBUG("{}: Initializing Node", nameId());

    // Initialize connected Nodes
    for (const auto& inputPin : inputPins)
    {
        if (inputPin.type != Pin::Type::Flow)
        {
            if (Node* connectedNode = nm::FindConnectedNodeToInputPin(inputPin.id))
            {
                if (!connectedNode->isInitialized() && !connectedNode->isInitializing())
                {
                    LOG_DEBUG("{}: Initializing connected Node '{}' on input Pin {}", nameId(), connectedNode->nameId(), size_t(inputPin.id));
                    if (!connectedNode->initializeNode())
                    {
                        LOG_ERROR("{}: Could not initialize connected node {}", nameId(), connectedNode->nameId());
                        isInitializing_ = false;
                        return false;
                    }
                }
            }
        }
    }

    // Initialize the node itself
    isInitialized_ = initialize();

    isInitializing_ = false;

    return isInitialized();
}

void NAV::Node::deinitializeNode()
{
    // Lock the node against recursive calling
    isDeinitializing_ = true;

    LOG_DEBUG("{}: Deinitializing Node", nameId());

    callbacksEnabled = false;

    // Deinitialize connected Nodes
    for (const auto& outputPin : outputPins)
    {
        if (outputPin.type != Pin::Type::Flow)
        {
            auto connectedNodes = nm::FindConnectedNodesToOutputPin(outputPin.id);
            for (auto* connectedNode : connectedNodes)
            {
                if (connectedNode->isInitialized() && !connectedNode->isDeinitializing())
                {
                    LOG_DEBUG("{}: Deinitializing connected Node '{}' on output Pin {}", nameId(), connectedNode->nameId(), size_t(outputPin.id));
                    connectedNode->deinitializeNode();
                }
            }
        }
    }

    // Deinitialize the node itself
    deinitialize();
    isInitialized_ = false;

    isDeinitializing_ = false;
}

bool NAV::Node::initialize()
{
    return enabled;
}

void NAV::Node::deinitialize() {}

void NAV::Node::flush() {}

bool NAV::Node::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    return initialize();
}

bool NAV::Node::onCreateLink(Pin* /*startPin*/, Pin* /*endPin*/)
{
    return true;
}

void NAV::Node::onDeleteLink(Pin* /*startPin*/, Pin* /*endPin*/) {}

void NAV::Node::afterCreateLink(Pin* /*startPin*/, Pin* /*endPin*/) {}

void NAV::Node::afterDeleteLink(Pin* /*startPin*/, Pin* /*endPin*/) {}

void NAV::Node::notifyOnOutputValueChanged(ax::NodeEditor::LinkId /*linkId*/) {}

void NAV::Node::notifyInputValueChanged(size_t portIndex)
{
    if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.at(portIndex).id))
    {
        if (nm::showFlowWhenNotifyingValueChange)
        {
            ax::NodeEditor::Flow(connectedLink->id, true);
        }

        if (Pin* startPin = nm::FindPin(connectedLink->startPinId))
        {
            if (startPin->parentNode && startPin->parentNode->enabled)
            {
                // Notify the node itself that changes were made
                startPin->parentNode->notifyOnOutputValueChanged(connectedLink->id);
                // Notify all nodes which registered a notify callback
                for (auto& [node, callback, linkId] : startPin->notifyFunc)
                {
                    if (node->id == id)
                    {
                        continue;
                    }
                    if (nm::showFlowWhenNotifyingValueChange)
                    {
                        ax::NodeEditor::Flow(linkId);
                    }

                    std::invoke(callback, node, linkId);
                }
            }
        }
    }
}

void NAV::Node::notifyOutputValueChanged(size_t portIndex)
{
    for (auto& [node, callback, linkId] : outputPins.at(portIndex).notifyFunc)
    {
        if (!node->enabled)
        {
            continue;
        }

        if (nm::showFlowWhenNotifyingValueChange)
        {
            ax::NodeEditor::Flow(linkId);
        }

        std::invoke(callback, node, linkId);
    }
}

void NAV::Node::invokeCallbacks(size_t portIndex, const std::shared_ptr<NAV::NodeData>& data)
{
    if (callbacksEnabled)
    {
        for (auto& [node, callback, linkId] : outputPins.at(portIndex).callbacks)
        {
            if (node->enabled && node->isInitialized())
            {
                if (nm::showFlowWhenInvokingCallbacks)
                {
                    ax::NodeEditor::Flow(linkId);
                }

                std::invoke(callback, node, data, linkId);
            }
        }
    }
}

size_t NAV::Node::pinIndexFromId(ax::NodeEditor::PinId pinId) const
{
    for (size_t i = 0; i < inputPins.size(); i++)
    {
        if (pinId == inputPins.at(i).id)
        {
            return i;
        }
    }
    for (size_t i = 0; i < outputPins.size(); i++)
    {
        if (pinId == outputPins.at(i).id)
        {
            return i;
        }
    }

    throw std::runtime_error(fmt::format("{}: The Pin {} is not on this node.", nameId(), size_t(pinId)).c_str());
}

std::string NAV::Node::nameId() const
{
    return fmt::format("{} ({})", name, size_t(id));
}

bool NAV::Node::isInitialized() const
{
    return isInitialized_;
}

bool NAV::Node::isInitializing() const
{
    return isInitializing_;
}

bool NAV::Node::isDeinitializing() const
{
    return isDeinitializing_;
}