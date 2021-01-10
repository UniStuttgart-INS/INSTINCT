#include "Node.hpp"

#include <stdexcept>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

void NAV::Node::guiConfig() {}

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

bool NAV::Node::onCreateLink(Pin* /*startPin*/, Pin* /*endPin*/)
{
    return true;
}

void NAV::Node::onDeleteLink(Pin* /*startPin*/, Pin* /*endPin*/) {}

void NAV::Node::invokeCallbacks(size_t portIndex, const std::shared_ptr<NAV::NodeData>& data)
{
    if (callbacksEnabled)
    {
        for (auto& [node, callback, linkId] : outputPins.at(portIndex).callbacks)
        {
            if (node->isInitialized)
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