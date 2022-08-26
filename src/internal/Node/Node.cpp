#include "Node.hpp"

#include <stdexcept>

#include "util/StringUtil.hpp"
#include "util/Assert.h"

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

bool NAV::Node::onCreateLink(OutputPin& /*startPin*/, InputPin& /*endPin*/)
{
    return true;
}

void NAV::Node::onDeleteLink(OutputPin& /*startPin*/, InputPin& /*endPin*/) {}

void NAV::Node::afterCreateLink(OutputPin& /*startPin*/, InputPin& /*endPin*/) {}

void NAV::Node::afterDeleteLink(OutputPin& /*startPin*/, InputPin& /*endPin*/) {}

void NAV::Node::notifyOutputValueChanged(size_t /* portIndex */)
{
    // TODO: Refactor this

    // for (auto& [node, callback, linkId] : outputPins.at(portIndex).notifyFuncOld)
    // {
    //     if (node->isDisabled())
    //     {
    //         continue;
    //     }

    //     if (nm::showFlowWhenNotifyingValueChange)
    //     {
    //         FlowAnimation::Add(linkId);
    //     }

    //     // TODO: Put this into the Callback Manager
    //     std::invoke(callback, node, linkId);
    // }
}

void NAV::Node::invokeCallbacks(size_t /* portIndex */, const std::shared_ptr<const NAV::NodeData>& /* data */)
{
    //     if (callbacksEnabled)
    //     {
    //         if (data == nullptr)
    //         {
    //             LOG_DEBUG("{}: Tried to invokeCallbacks on pin {} with a nullptr. This is a bug!!!", nameId(), portIndex);
    //             return;
    //         }
    // #ifdef TESTING
    //         for (const auto& watcherCallback : outputPins.at(portIndex).watcherCallbacksOld)
    //         {
    //             const auto& linkId = watcherCallback.second;
    //             if (!linkId) // Trigger all output pin callbacks
    //             {
    //                 CallbackManager::queueWatcherCallbackForInvocation(watcherCallback, data);
    //             }
    //         }
    // #endif

    //         for (const auto& nodeCallback : outputPins.at(portIndex).callbacksOld)
    //         {
    //             const auto* node = std::get<0>(nodeCallback);

    //             if (node->isInitialized())
    //             {
    // #ifdef TESTING
    //                 const auto& linkId = std::get<2>(nodeCallback);
    //                 for (const auto& watcherCallback : outputPins.at(portIndex).watcherCallbacksOld)
    //                 {
    //                     const auto& watcherLinkId = watcherCallback.second;
    //                     if (linkId == watcherLinkId)
    //                     {
    //                         CallbackManager::queueWatcherCallbackForInvocation(watcherCallback, data);
    //                     }
    //                 }
    // #endif
    //                 CallbackManager::queueNodeCallbackForInvocation(nodeCallback, data);
    //             }
    //         }

    //         while (CallbackManager::hasUnprocessedCallbacks())
    //         {
    //             CallbackManager::processNextCallback();
    //         }
    //     }
}

NAV::InputPin& NAV::Node::inputPinFromId(ax::NodeEditor::PinId pinId)
{
    for (auto& inputPin : inputPins)
    {
        if (pinId == inputPin.id) { return inputPin; }
    }

    throw std::runtime_error(fmt::format("{}: The Pin {} is not on this node.", nameId(), size_t(pinId)).c_str());
}

NAV::OutputPin& NAV::Node::outputPinFromId(ax::NodeEditor::PinId pinId)
{
    for (auto& outputPin : outputPins)
    {
        if (pinId == outputPin.id) { return outputPin; }
    }

    throw std::runtime_error(fmt::format("{}: The Pin {} is not on this node.", nameId(), size_t(pinId)).c_str());
}

size_t NAV::Node::inputPinIndexFromId(ax::NodeEditor::PinId pinId) const
{
    for (size_t i = 0; i < inputPins.size(); i++)
    {
        if (pinId == inputPins.at(i).id) { return i; }
    }

    throw std::runtime_error(fmt::format("{}: The Pin {} is not on this node.", nameId(), size_t(pinId)).c_str());
}

size_t NAV::Node::outputPinIndexFromId(ax::NodeEditor::PinId pinId) const
{
    for (size_t i = 0; i < outputPins.size(); i++)
    {
        if (pinId == outputPins.at(i).id) { return i; }
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
    LOG_TRACE("{}: Current state = {}", nameId(), toString(_state));

    switch (_state)
    {
    case State::Initialized:
        return true;
    case State::Disabled:
    case State::DoShutdown:
    case State::Shutdown:
        return false;
    case State::DoDeinitialize:
    case State::Deinitializing:
        if (_reinitialize) { break; }
        return false;
    case State::DoInitialize:
    case State::Initializing:
        break;
    case State::Deinitialized:
    {
        {
            std::lock_guard lk(_workerMutex);
            _state = State::DoInitialize;
        }
        _workerConditionVariable.notify_all();
        break;
    }
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Initialized || _state == State::Deinitialized; });
        return _state == State::Initialized;
    }
    return true;
}

bool NAV::Node::doReinitialize(bool wait)
{
    LOG_TRACE("{}: Current state = {}", nameId(), toString(_state));

    switch (_state)
    {
    case State::Disabled:
    case State::DoShutdown:
    case State::Shutdown:
        return false;
    case State::DoDeinitialize:
    case State::Deinitializing:
        _reinitialize = true;
        break;
    case State::DoInitialize:
    case State::Initializing:
    case State::Deinitialized:
        break;
    case State::Initialized:
    {
        {
            std::lock_guard lk(_workerMutex);
            _state = State::DoDeinitialize;
            _reinitialize = true;
        }
        _workerConditionVariable.notify_all();
        break;
    }
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
    LOG_TRACE("{}: Current state = {}", nameId(), toString(_state));

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
    {
        {
            std::lock_guard lk(_workerMutex);
            _state = State::DoDeinitialize;
        }
        _workerConditionVariable.notify_all();
        break;
    }
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Deinitialized; });
    }
    return true;
}

bool NAV::Node::doDisable(bool wait)
{
    LOG_TRACE("{}: Current state = {}", nameId(), toString(_state));

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
        _disable = true;
        doDeinitialize();
        break;
    case State::DoInitialize:
    case State::Deinitialized:
    {
        {
            std::lock_guard lk(_workerMutex);
            _state = State::Disabled;
        }
        break;
    }
    }

    if (wait)
    {
        std::unique_lock lk(_workerMutex);
        _workerConditionVariable.wait(lk, [&, this] { return _state == State::Deinitialized; });
    }
    return true;
}

bool NAV::Node::doEnable()
{
    LOG_TRACE("{}: Current state = {}", nameId(), toString(_state));

    if (_state == State::Disabled)
    {
        std::lock_guard lk(_workerMutex);
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
            {
                std::lock_guard lk(node->_workerMutex);
                node->_state = State::Shutdown;
            }
            node->_workerConditionVariable.notify_all();
            break;
        }
        if (node->_state == State::DoInitialize)
        {
            LOG_TRACE("{}: Worker doing initialization...", node->nameId());
            node->workerInitializeNode();
            LOG_TRACE("{}: Worker finished initialization, notifying all waiting threads.", node->nameId());
            node->_workerConditionVariable.notify_all();
            continue;
        }
        if (node->_state == State::DoDeinitialize)
        {
            LOG_TRACE("{}: Worker doing deinitialization...", node->nameId());
            node->workerDeinitializeNode();
            LOG_TRACE("{}: Worker finished deinitialization, notifying all waiting threads.", node->nameId());
            node->_workerConditionVariable.notify_all();
            continue;
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

    INS_ASSERT_USER_ERROR(_state == State::DoInitialize, fmt::format("Worker can only initialize the node if the state is set to DoInitialize, but it is {}.", toString(_state)).c_str());
    {
        std::lock_guard lk(_workerMutex);
        _state = Node::State::Initializing;
    }
    LOG_DEBUG("{}: Initializing Node", nameId());

    // Initialize Nodes connected to the input pins
    for (const auto& inputPin : inputPins)
    {
        if (inputPin.type != Pin::Type::Flow)
        {
            if (Node* connectedNode = inputPin.link.connectedNode)
            {
                if (!connectedNode->isInitialized())
                {
                    LOG_DEBUG("{}: Initializing connected Node '{}' on input Pin {}", nameId(), connectedNode->nameId(), size_t(inputPin.id));
                    if (!connectedNode->doInitialize(true))
                    {
                        LOG_ERROR("{}: Could not initialize connected node {}", nameId(), connectedNode->nameId());
                        if (_state == State::Initializing)
                        {
                            std::lock_guard lk(_workerMutex);
                            _state = Node::State::Deinitialized;
                        }
                        return false;
                    }
                }
            }
        }
    }

    _reinitialize = false;

    // Initialize the node itself
    if (initialize())
    {
        if (_state == State::Initializing)
        {
            std::lock_guard lk(_workerMutex);
            _state = Node::State::Initialized;
        }
        return true;
    }

    if (_state == State::Initializing)
    {
        std::lock_guard lk(_workerMutex);
        _state = Node::State::Deinitialized;
    }
    return false;
}

bool NAV::Node::workerDeinitializeNode()
{
    LOG_TRACE("{}: called", nameId());

    INS_ASSERT_USER_ERROR(_state == State::DoDeinitialize, fmt::format("Worker can only deinitialize the node if the state is set to DoDeinitialize, but it is {}.", toString(_state)).c_str());
    {
        std::lock_guard lk(_workerMutex);
        _state = Node::State::Deinitializing;
    }
    LOG_DEBUG("{}: Deinitializing Node", nameId());

    callbacksEnabled = false;

    // Re-/Deinitialize Nodes connected to the output pins
    for (const auto& outputPin : outputPins)
    {
        if (outputPin.type != Pin::Type::Flow)
        {
            for (const auto& link : outputPin.links)
            {
                if (link.connectedNode->isInitialized())
                {
                    LOG_DEBUG("{}: {} connected Node '{}' on output Pin {}", nameId(),
                              _reinitialize ? "Reinitializing" : "Deinitializing", link.connectedNode->nameId(), size_t(outputPin.id));
                    if (_reinitialize) { link.connectedNode->doReinitialize(); }
                    else { link.connectedNode->doDeinitialize(); }
                }
            }
        }
    }

    // Deinitialize the node itself
    deinitialize();

    if (_state == State::Deinitializing)
    {
        if (_disable)
        {
            std::lock_guard lk(_workerMutex);
            _state = State::Disabled;
        }
        else if (_reinitialize)
        {
            std::lock_guard lk(_workerMutex);
            _state = State::DoInitialize;
        }
        else
        {
            std::lock_guard lk(_workerMutex);
            _state = State::Deinitialized;
        }
    }

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
            std::lock_guard lk(node._workerMutex);
            node._state = Node::State::Disabled;
        }
    }

    if (j.contains("inputPins"))
    {
        auto inputPins = j.at("inputPins").get<std::vector<InputPin>>();
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
        auto outputPins = j.at("outputPins").get<std::vector<OutputPin>>();
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
