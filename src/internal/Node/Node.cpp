#include "Node.hpp"

#include <stdexcept>

#include "util/StringUtil.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/CallbackManager.hpp"
#include "internal/gui/FlowAnimation.hpp"
#include "util/Json.hpp"

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

void NAV::Node::guiConfig() {}

json NAV::Node::save() const { return {}; }

void NAV::Node::restore(const json& /*j*/) {}

void NAV::Node::restoreAtferLink(const json& /*j*/) {}

bool NAV::Node::initializeNode()
{
    if (!_isEnabled)
    {
        return false;
    }

    // Lock the node against recursive calling
    _isInitializing = true;

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
                        _isInitializing = false;
                        return false;
                    }
                }
            }
        }
    }

    // Initialize the node itself
    _isInitialized = initialize();

    _isInitializing = false;

    return isInitialized();
}

void NAV::Node::deinitializeNode()
{
    if (isDeinitializing())
    {
        return;
    }

    // Lock the node against recursive calling
    _isDeinitializing = true;

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
    _isInitialized = false;

    _isDeinitializing = false;
}

bool NAV::Node::initialize()
{
    return _isEnabled;
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
            FlowAnimation::Add(connectedLink->id, ax::NodeEditor::FlowDirection::Backward);
        }

        if (Pin* startPin = nm::FindPin(connectedLink->startPinId))
        {
            if (startPin->parentNode && startPin->parentNode->_isEnabled)
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
                        FlowAnimation::Add(linkId);
                    }
                    // TODO: Put this into the Callback Manager
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
        if (!node->_isEnabled)
        {
            continue;
        }

        if (nm::showFlowWhenNotifyingValueChange)
        {
            FlowAnimation::Add(linkId);
        }

        // TODO: Put this into the Callback Manager
        std::invoke(callback, node, linkId);
    }
}

void NAV::Node::invokeCallbacks(size_t portIndex, const std::shared_ptr<const NAV::NodeData>& data)
{
    if (callbacksEnabled)
    {
#ifdef TESTING
        for (const auto& watcherCallback : outputPins.at(portIndex).watcherCallbacks)
        {
            const auto& linkId = watcherCallback.second;
            if (!linkId) // Trigger all output pin callbacks
            {
                CallbackManager::queueWatcherCallbackForInvocation(watcherCallback, data);
            }
        }
#endif

        for (const auto& nodeCallback : outputPins.at(portIndex).callbacks)
        {
            const auto* node = std::get<0>(nodeCallback);

            if (node->_isEnabled && node->isInitialized())
            {
#ifdef TESTING
                const auto& linkId = std::get<2>(nodeCallback);
                for (const auto& watcherCallback : outputPins.at(portIndex).watcherCallbacks)
                {
                    const auto& watcherLinkId = watcherCallback.second;
                    if (linkId == watcherLinkId)
                    {
                        CallbackManager::queueWatcherCallbackForInvocation(watcherCallback, data);
                    }
                }
#endif
                CallbackManager::queueNodeCallbackForInvocation(nodeCallback, data);
            }
        }

        while (CallbackManager::hasUnprocessedCallbacks())
        {
            CallbackManager::processNextCallback();
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
    return fmt::format("{} ({})", str::replaceAll_copy(name, "\n", ""), size_t(id));
}

bool NAV::Node::isInitialized() const
{
    return _isInitialized;
}

bool NAV::Node::isInitializing() const
{
    return _isInitializing;
}

bool NAV::Node::isDeinitializing() const
{
    return _isDeinitializing;
}

bool NAV::Node::isEnabled() const
{
    return _isEnabled;
}

const ImVec2& NAV::Node::getSize() const
{
    return _size;
}

void NAV::to_json(json& j, const Node& node)
{
    ImVec2 realSize = ed::GetNodeSize(node.id);
    realSize.x -= 16;
    realSize.y -= 38;
    j = json{
        { "id", size_t(node.id) },
        { "type", node.type() },
        { "kind", std::string(node.kind) },
        { "name", node.name },
        { "size", node._size.x == 0 && node._size.y == 0 ? node._size : realSize },
        { "pos", ed::GetNodePosition(node.id) },
        { "enabled", node._isEnabled },
        { "inputPins", node.inputPins },
        { "outputPins", node.outputPins },
    };
}
void NAV::from_json(const json& j, Node& node)
{
    node.id = j.at("id").get<size_t>();
    if (j.contains("kind"))
    {
        node.kind = Node::Kind(j.at("kind").get<std::string>());
    }
    if (j.contains("name"))
    {
        j.at("name").get_to(node.name);
    }
    if (j.contains("size"))
    {
        j.at("size").get_to(node._size);
    }
    if (j.contains("enabled"))
    {
        j.at("enabled").get_to(node._isEnabled);
    }

    if (j.contains("inputPins"))
    {
        auto inputPins = j.at("inputPins").get<std::vector<Pin>>();
        for (size_t i = 0; i < inputPins.size(); ++i)
        {
            if (node.inputPins.size() <= i)
            {
                break;
            }
            node.inputPins.at(i).id = inputPins.at(i).id;
            // node.inputPins.at(i).type = inputPins.at(i).type;
            // node.inputPins.at(i).name = inputPins.at(i).name;
            // node.inputPins.at(i).dataIdentifier = inputPins.at(i).dataIdentifier;
        }
    }

    if (j.contains("outputPins"))
    {
        auto outputPins = j.at("outputPins").get<std::vector<Pin>>();
        for (size_t i = 0; i < outputPins.size(); ++i)
        {
            if (node.outputPins.size() <= i)
            {
                break;
            }
            node.outputPins.at(i).id = outputPins.at(i).id;
            // node.outputPins.at(i).type = outputPins.at(i).type;
            // node.outputPins.at(i).name = outputPins.at(i).name;
            // node.outputPins.at(i).dataIdentifier = outputPins.at(i).dataIdentifier;
        }
    }
}
