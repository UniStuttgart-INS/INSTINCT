#include "Node.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

bool NAV::Node::initialize()
{
    // Lock the node against recursive calling
    isInitializing = true;

    for (const auto& inputPin : inputPins)
    {
        if (inputPin.type != Pin::Type::Flow)
        {
            auto connectedNodes = nm::FindConnectedNodesToPin(inputPin.id);
            if (!connectedNodes.empty())
            {
                Node* connectedNode = connectedNodes.front();
                if (!connectedNode->isInitialized && !connectedNode->isInitializing)
                {
                    if (!connectedNode->initialize())
                    {
                        LOG_ERROR("{}: Could not initialize connected node {}", nameId(), connectedNode->nameId());
                        isInitializing = false;
                        return false;
                    }
                }
            }
        }
    }

    for (const auto& outputPin : outputPins)
    {
        auto connectedNodes = nm::FindConnectedNodesToPin(outputPin.id);
        if (!connectedNodes.empty())
        {
            Node* connectedNode = connectedNodes.front();
            if (!connectedNode->isInitialized && !connectedNode->isInitializing)
            {
                if (!connectedNode->initialize())
                {
                    LOG_WARN("{}: Could not initialize connected node {}", nameId(), connectedNode->nameId());
                }
            }
        }
    }

    isInitializing = false;

    return true;
}

void NAV::Node::deinitialize()
{
    callbacksEnabled = false;
    isInitialized = false;
}

void NAV::Node::resetNode() {}

void NAV::Node::invokeCallbacks(size_t portIndex, const std::shared_ptr<NAV::NodeData>& data)
{
    if (callbacksEnabled)
    {
        if (nm::showFlowWhenInvokingCallbacks)
        {
            auto connectedLinks = nm::FindConnectedLinksToPin(outputPins.at(portIndex).id);
            for (auto& connectedLink : connectedLinks)
            {
                ax::NodeEditor::Flow(connectedLink->id);
            }
        }

        for (auto& [node, callback] : outputPins.at(portIndex).callbacks)
        {
            std::invoke(callback, node, data);
        }
    }
}