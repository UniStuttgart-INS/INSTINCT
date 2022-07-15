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

NAV::Node::Node(std::string name)
    : name(std::move(name))
{
    LOG_TRACE("{}: called", nameId());
    _worker = std::thread(workerThread, this);
}

NAV::Node::~Node()
{
    LOG_TRACE("{}: called", nameId());
    {
        std::lock_guard lk(_workerMutex);
        _state = State::DoShutdown;
        _workerConditionVariable.notify_all();
    }
    // // wait for the worker
    // {
    //     std::unique_lock lk(_workerMutex);
    //     _workerConditionVariable.wait(lk, [&, this] { return _state == State::Shutdown; });
    // }
    _worker.join();
}

void NAV::Node::guiConfig() {}

json NAV::Node::save() const { return {}; }

void NAV::Node::restore(const json& /*j*/) {}

void NAV::Node::restoreAtferLink(const json& /*j*/) {}

bool NAV::Node::initialize()
{
    return true;
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
            if (startPin->parentNode && !startPin->parentNode->isDisabled())
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
        if (node->isDisabled())
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
        if (data == nullptr)
        {
            LOG_DEBUG("{}: Tried to invokeCallbacks on pin {} with a nullptr. This is a bug!!!", nameId(), portIndex);
            return;
        }
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

            if (node->isInitialized())
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

const ImVec2& NAV::Node::getSize() const
{
    return _size;
}

std::string NAV::Node::toString(State state)
{
    switch (state)
    {
    case State::Disabled:
        return "Disabled";
    case State::Deinitialized:
        return "Deinitialized";
    case State::DoInitialize:
        return "DoInitialize";
    case State::Initializing:
        return "Initializing";
    case State::Initialized:
        return "Initialized";
    case State::DoDeinitialize:
        return "DoDeinitialize";
    case State::Deinitializing:
        return "Deinitializing";
    case State::DoShutdown:
        return "DoShutdown";
    case State::Shutdown:
        return "Shutdown";
    }
    return "";
}

NAV::Node::State NAV::Node::getState() const
{
    return _state;
}

bool NAV::Node::doInitialize(bool wait)
{
    LOG_TRACE("{}: Current state = {}", nameId(), _state);

    switch (_state)
    {
    case State::Initialized:
        return true;
    case State::Disabled:
    case State::DoShutdown:
    case State::Shutdown:
    case State::DoDeinitialize:
    case State::Deinitializing:
        return false;
    case State::DoInitialize:
    case State::Initializing:
        break;
    case State::Deinitialized:
        _state = State::DoInitialize;
        break;
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Initialized || _state == State::Deinitialized; });
        return _state == State::Initialized;
    }
    return true;
}

bool NAV::Node::doDeinitialize(bool wait)
{
    LOG_TRACE("{}: Current state = {}", nameId(), _state);

    switch (_state)
    {
    case State::Deinitialized:
        return true;
    case State::Disabled:
    case State::DoShutdown:
    case State::Shutdown:
    case State::DoInitialize:
    case State::Initializing:
        return false;
    case State::DoDeinitialize:
    case State::Deinitializing:
        break;
    case State::Initialized:
        _state = State::DoDeinitialize;
        break;
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Deinitialized; });
    }
    return true;
}

bool NAV::Node::doDisableNode(bool wait)
{
    LOG_TRACE("{}: Current state = {}", nameId(), _state);

    switch (_state)
    {
    case State::Disabled:
        return true;
    case State::DoShutdown:
    case State::Shutdown:
    case State::Initializing:
        return false;
    case State::Initialized:
    case State::DoDeinitialize:
    case State::Deinitializing:
        doDeinitialize();
        break;
    case State::DoInitialize:
    case State::Deinitialized:
        break;
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Deinitialized; });
    }
    _state = State::Disabled;
    return true;
}

bool NAV::Node::doEnableNode()
{
    LOG_TRACE("{}: Current state = {}", nameId(), _state);

    if (_state == State::Disabled)
    {
        _state = State::Deinitialized;
    }
    return true;
}

bool NAV::Node::isDisabled() const
{
    return _state == State::Disabled;
}
bool NAV::Node::isInitialized() const
{
    return _state == State::Initialized;
}

void NAV::Node::workerThread(Node* node)
{
    LOG_TRACE("{}: Worker thread started.", node->nameId());

    while (node->_state != State::Shutdown)
    {
        if (node->_state == State::DoShutdown)
        {
            LOG_TRACE("{}: Worker doing shutdown...", node->nameId());
            node->_state = State::Shutdown;
            node->_workerConditionVariable.notify_all();
            break;
        }

        std::unique_lock lk(node->_workerMutex);
        std::cv_status status = node->_workerConditionVariable.wait_for(lk, node->_workerTimeout);

        LOG_TRACE("{}: Worker waking up...", node->nameId());

        if (status == std::cv_status::timeout)
        {
            node->workerTimeoutHandler();
        }
        else // Triggered by '_workerConditionVariable.notify_all();'
        {
            if (node->_state == State::DoInitialize)
            {
                LOG_TRACE("{}: Worker doing initialization...", node->nameId());
                node->workerInitializeNode();
                // Manual unlocking is done before notifying, to avoid waking up
                // the waiting thread only to block again (see notify_one for details)
                lk.unlock();
                node->_workerConditionVariable.notify_all();
            }
            else if (node->_state == State::DoDeinitialize)
            {
                LOG_TRACE("{}: Worker doing deinitialization...", node->nameId());
                node->workerDeinitializeNode();
                lk.unlock();
                node->_workerConditionVariable.notify_all();
            }

            if (node->isInitialized())
            {
                // TODO: Handle data here
            }
        }
    }

    LOG_TRACE("{}: Worker thread ended.", node->nameId());
}

bool NAV::Node::workerInitializeNode()
{
    LOG_TRACE("{}: called", nameId());
    if (isDisabled())
    {
        return false;
    }

    if (isInitialized())
    {
        if (resetNode())
        {
            return true;
        }
        _state = State::DoDeinitialize;
        _reinitialize = true;
    }

    if (_state != Node::State::Deinitialized && _state != Node::State::DoInitialize)
    {
        return false;
    }

    // Lock the node against recursive calling
    _state = Node::State::Initializing;

    LOG_DEBUG("{}: Initializing Node", nameId());

    // Initialize connected Nodes
    for (const auto& inputPin : inputPins)
    {
        if (inputPin.type != Pin::Type::Flow)
        {
            if (Node* connectedNode = nm::FindConnectedNodeToInputPin(inputPin.id))
            {
                if (!connectedNode->isInitialized())
                {
                    LOG_DEBUG("{}: Initializing connected Node '{}' on input Pin {}", nameId(), connectedNode->nameId(), size_t(inputPin.id));
                    if (!connectedNode->doInitialize(true))
                    {
                        LOG_ERROR("{}: Could not initialize connected node {}", nameId(), connectedNode->nameId());
                        _state = Node::State::Deinitialized;
                        return false;
                    }
                }
            }
        }
    }

    // Initialize the node itself
    if (initialize())
    {
        _state = Node::State::Initialized;

        if (_reinitialize)
        {
            // Reinitialize connected Nodes
            for (const auto& outputPin : outputPins)
            {
                if (outputPin.type != Pin::Type::Flow)
                {
                    auto connectedNodes = nm::FindConnectedNodesToOutputPin(outputPin.id);
                    for (auto* connectedNode : connectedNodes)
                    {
                        if (!connectedNode->isInitialized())
                        {
                            LOG_DEBUG("{}: Reinitializing connected Node '{}' on output Pin {}", nameId(), connectedNode->nameId(), size_t(outputPin.id));
                            connectedNode->doInitialize(true);
                        }
                    }
                }
            }
            _reinitialize = false;
        }
        return true;
    }

    _state = Node::State::Deinitialized;
    return false;
}

bool NAV::Node::workerDeinitializeNode()
{
    LOG_TRACE("{}: called", nameId());
    if (_state == Node::State::Deinitializing || _state == Node::State::Deinitialized)
    {
        return false;
    }

    // Lock the node against recursive calling
    _state = Node::State::Deinitializing;

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
                if (connectedNode->isInitialized())
                {
                    LOG_DEBUG("{}: Deinitializing connected Node '{}' on output Pin {}", nameId(), connectedNode->nameId(), size_t(outputPin.id));
                    connectedNode->doDeinitialize(true);
                }
            }
        }
    }

    // Deinitialize the node itself
    deinitialize();

    if (_reinitialize) { _state = Node::State::DoInitialize; }
    else { _state = Node::State::Deinitialized; }

    return true;
}

void NAV::Node::workerTimeoutHandler()
{
    LOG_TRACE("{}: called", nameId());
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
        { "enabled", !node.isDisabled() },
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
        bool enabled = j.at("enabled").get<bool>();
        if (!enabled)
        {
            node._state = Node::State::Disabled;
        }
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
            j.at("inputPins").at(i).get_to(node.inputPins.at(i));
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
            j.at("outputPins").at(i).get_to(node.outputPins.at(i));
        }
    }
}
