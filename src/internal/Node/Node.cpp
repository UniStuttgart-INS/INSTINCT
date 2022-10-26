// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Node.hpp"

#include <stdexcept>

#include "util/StringUtil.hpp"
#include "util/Assert.h"

#include "internal/FlowExecutor.hpp"
#include "internal/gui/FlowAnimation.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "util/Json.hpp"

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

NAV::Node::Node(std::string name)
    : name(std::move(name))
{
    LOG_TRACE("{}: called", nameId());
    if (_autostartWorker)
    {
        _worker = std::thread(workerThread, this);
    }
}

NAV::Node::~Node()
{
    LOG_TRACE("{}: called", nameId());

    if (_autostartWorker)
    {
        _state = State::DoShutdown;
        wakeWorker();

        // // wait for the worker
        // {
        //     std::unique_lock lk(_workerMutex);
        //     _workerConditionVariable.wait(lk, [&, this] { return _state == State::Shutdown; });
        // }
        _worker.join();
    }
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

void NAV::Node::notifyOutputValueChanged(size_t portIndex, const InsTime& insTime)
{
    if (callbacksEnabled && isInitialized())
    {
        auto& outputPin = outputPins.at(portIndex);

        if (!outputPin.isPinLinked()) { return; }

        bool mutexLocked = false;
        for (const auto& link : outputPin.links)
        {
            auto* targetPin = link.getConnectedPin();
            if (link.connectedNode->isInitialized() && !targetPin->queueBlocked)
            {
                if (!mutexLocked)
                {
                    mutexLocked = true;
                    outputPin.dataAccessMutex.lock();
                }
                outputPin.dataAccessCounter++;

                if (nm::showFlowWhenNotifyingValueChange)
                {
                    FlowAnimation::Add(link.linkId);
                }

                auto data = std::make_shared<NodeData>();
                data->insTime = insTime;

                targetPin->queue.push_back(data);
            }
        }
        for (const auto& link : outputPin.links)
        {
            auto* targetPin = link.getConnectedPin();
            if (link.connectedNode->isInitialized() && !targetPin->queueBlocked)
            {
                LOG_DATA("{}: Waking up worker of node '{}'. New data on pin '{}'", nameId(), link.connectedNode->nameId(), targetPin->name);
                link.connectedNode->wakeWorker();
            }
        }
    }
}

std::mutex* NAV::Node::getInputValueMutex(size_t portIndex)
{
    if (OutputPin* outputPin = inputPins.at(portIndex).link.getConnectedPin())
    {
        return &outputPin->dataAccessMutex;
    }
    return nullptr;
}

void NAV::Node::releaseInputValue(size_t portIndex)
{
    OutputPin* outputPin = inputPins.at(portIndex).link.getConnectedPin();
    if (outputPin && outputPin->dataAccessCounter > 0)
    {
        outputPin->dataAccessCounter--;
        if (outputPin->dataAccessCounter == 0)
        {
            LOG_DATA("{}: Unlocking dataAccessMutex of pin '{}'", nameId(), outputPin->name);
            outputPin->dataAccessMutex.unlock();
        }
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

        for (const auto& link : outputPins.at(portIndex).links)
        {
            auto* targetPin = link.getConnectedPin();
            if (link.connectedNode->isInitialized() && !targetPin->queueBlocked)
            {
                if (NodeManager::showFlowWhenNotifyingValueChange)
                {
                    FlowAnimation::Add(link.linkId);
                }

                targetPin->queue.push_back(data);
                LOG_DATA("{}: Waking up worker of node '{}'. New data on pin '{}'", nameId(), link.connectedNode->nameId(), targetPin->name);
                link.connectedNode->wakeWorker();
            }
        }
    }
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

NAV::Node::Mode NAV::Node::getMode() const
{
    return _mode;
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
        _state = State::DoInitialize;
        wakeWorker();
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
        _state = State::DoDeinitialize;
        _reinitialize = true;
        wakeWorker();
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
        _state = State::DoDeinitialize;
        wakeWorker();
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
        _state = State::Disabled;
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
        _state = State::Deinitialized;
    }
    return true;
}

void NAV::Node::wakeWorker()
{
    {
        std::lock_guard lk(_workerMutex);
        _workerWakeup = true;
    }
    _workerConditionVariable.notify_all();
}

bool NAV::Node::isDisabled() const
{
    return _state == State::Disabled;
}
bool NAV::Node::isInitialized() const
{
    return _state == State::Initialized;
}
bool NAV::Node::isTransient() const
{
    switch (_state)
    {
    case State::Disabled:
    case State::Initialized:
    case State::Deinitialized:
        return false;
    case State::DoShutdown:
    case State::Shutdown:
    case State::Initializing:
    case State::DoDeinitialize:
    case State::Deinitializing:
    case State::DoInitialize:
        return true;
    }

    return true;
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
        if (node->_state == State::DoInitialize)
        {
            LOG_TRACE("{}: Worker doing initialization...", node->nameId());
            node->workerInitializeNode();
            LOG_TRACE("{}: Worker finished initialization, notifying all waiting threads (state = {})", node->nameId(), Node::toString(node->_state));
            node->_workerConditionVariable.notify_all();
            continue;
        }
        if (node->_state == State::DoDeinitialize)
        {
            LOG_TRACE("{}: Worker doing deinitialization...", node->nameId());
            node->workerDeinitializeNode();
            LOG_TRACE("{}: Worker finished deinitialization, notifying all waiting threads (state = {})", node->nameId(), Node::toString(node->_state));
            node->_workerConditionVariable.notify_all();
            continue;
        }

        if (!node->isTransient())
        {
            // Wait for data or state change
            LOG_DATA("{}: Worker going to sleep", node->nameId());
            std::unique_lock lk(node->_workerMutex);
            bool result = node->_workerConditionVariable.wait_for(lk, node->_workerTimeout, [node] { return node->_workerWakeup; });
            bool wakeup = node->_workerWakeup;
            node->_workerWakeup = false;
            lk.unlock();
            LOG_DATA("{}: Worker woke up", node->nameId());

            if (result != wakeup) // Timeout reached
            {
                node->workerTimeoutHandler();
            }

            if (node->isInitialized() && (node->callbacksEnabled || node->_mode == Node::Mode::REAL_TIME))
            {
                // Check input pin for data and trigger callbacks
                if (std::any_of(node->inputPins.begin(), node->inputPins.end(), [](const InputPin& inputPin) {
                        return inputPin.isPinLinked();
                    }))
                {
                    while (node->isInitialized())
                    {
                        // -------------------------- Data processing on input non-flow pins -----------------------------
                        bool notifyTriggered = false;
                        for (size_t i = 0; i < node->inputPins.size(); i++)
                        {
                            auto& inputPin = node->inputPins[i];
                            if (inputPin.type != Pin::Type::Flow && !inputPin.queue.empty())
                            {
                                if (auto callback = std::get<InputPin::DataChangedNotifyFunc>(inputPin.callback))
                                {
                                    LOG_DATA("{}: Invoking notify callback on input pin '{}'", node->nameId(), inputPin.name);
                                    InsTime insTime = inputPin.queue.extract_front()->insTime;
#ifdef TESTING
                                    for (const auto& watcherCallback : inputPin.watcherCallbacks)
                                    {
                                        if (auto watcherCall = std::get<InputPin::DataChangedWatcherNotifyFunc>(watcherCallback))
                                        {
                                            std::invoke(watcherCall, node, insTime, i);
                                        }
                                    }
#endif
                                    std::invoke(callback, node, insTime, i);
                                    notifyTriggered = true;
                                }
                            }
                        }
                        if (notifyTriggered) { continue; }

                        // ------------------------------ Process data on input flow pins --------------------------------
                        if (node->callbacksEnabled || node->_mode == Node::Mode::REAL_TIME)
                        {
                            LOG_DATA("{}: Checking for firable input pins", node->nameId());

                            if (node->_mode == Mode::POST_PROCESSING)
                            {
                                // Check if all input flow pins have data
                                bool allInputPinsHaveData = !node->inputPins.empty();
                                for (const auto& inputPin : node->inputPins)
                                {
                                    if (inputPin.type == Pin::Type::Flow && inputPin.neededForTemporalQueueCheck && !inputPin.queueBlocked && inputPin.queue.empty())
                                    {
                                        if (auto* connectedPin = inputPin.link.getConnectedPin();
                                            connectedPin && connectedPin->mode == OutputPin::Mode::POST_PROCESSING)
                                        {
                                            allInputPinsHaveData = false;
                                            break;
                                        }
                                    }
                                }
                                if (!allInputPinsHaveData)
                                {
                                    LOG_DATA("{}: Not all pins have data for temporal sorting", node->nameId());
                                    break;
                                }
                                LOG_DATA("{}: All pins have data for temporal sorting", node->nameId());
                            }

                            // Find pin with the earliest data
                            InsTime earliestTime;
                            size_t earliestInputPinIdx = 0;
                            int earliestInputPinPriority = -1000;
                            for (size_t i = 0; i < node->inputPins.size(); i++)
                            {
                                auto& inputPin = node->inputPins[i];
                                if (inputPin.type == Pin::Type::Flow && !inputPin.queue.empty()
                                    && (earliestTime.empty()
                                        || inputPin.queue.front()->insTime < earliestTime
                                        || (inputPin.queue.front()->insTime == earliestTime && inputPin.priority > earliestInputPinPriority)))
                                {
                                    earliestTime = inputPin.queue.front()->insTime;
                                    earliestInputPinIdx = i;
                                    earliestInputPinPriority = inputPin.priority;
                                }
                            }
                            if (earliestInputPinPriority == -1000) { break; }

                            auto& inputPin = node->inputPins[earliestInputPinIdx];
                            if (inputPin.firable && inputPin.firable(node, inputPin))
                            {
                                if (auto callback = std::get<InputPin::FlowFirableCallbackFunc>(inputPin.callback))
                                {
                                    LOG_DATA("{}: Invoking callback on input pin '{}'", node->nameId(), inputPin.name);
#ifdef TESTING
                                    for (const auto& watcherCallback : inputPin.watcherCallbacks)
                                    {
                                        if (auto watcherCall = std::get<InputPin::FlowFirableWatcherCallbackFunc>(watcherCallback))
                                        {
                                            std::invoke(watcherCall, node, inputPin.queue, earliestInputPinIdx);
                                        }
                                    }
#endif
                                    std::invoke(callback, node, inputPin.queue, earliestInputPinIdx);
                                }
                            }
                            else if (inputPin.dropQueueIfNotFirable)
                            {
                                LOG_DATA("{}: Dropping message on input pin '{}'", node->nameId(), inputPin.name);
                                inputPin.queue.pop_front();
                            }
                            else
                            {
                                LOG_DATA("{}: Skipping message on input pin '{}'", node->nameId(), inputPin.name);
                                break; // Do not drop an item, but put the worker to sleep
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                // Post-processing (FileReader/Simulator)
                if (node->_mode == Node::Mode::POST_PROCESSING && !node->pollEvents.empty())
                {
                    std::multimap<InsTime, OutputPin*>::iterator it;
                    while (it = node->pollEvents.begin(), it != node->pollEvents.end() && node->isInitialized() && node->callbacksEnabled)
                    {
                        OutputPin* outputPin = it->second;
                        Node* node = outputPin->parentNode;
                        auto* callback = std::get_if<OutputPin::PollDataFunc>(&outputPin->data);
                        if (callback != nullptr && *callback != nullptr)
                        {
                            if (!it->first.empty())
                            {
                                LOG_DATA("{}: Polling data from output pin '{}'", node->nameId(), str::replaceAll_copy(outputPin->name, "\n", ""));
                                // Trigger the already peeked observation and invoke it's callbacks (peek = false)
                                if ((node->**callback)(false) == nullptr)
                                {
                                    LOG_ERROR("{}: {} could not poll its observation despite being able to peek it.", node->nameId(), outputPin->name);
                                }
                            }

                            // Add next data event from the node
                            while (true)
                            {
                                // Check if data available (peek = true)
                                if (auto obs = (node->**callback)(true))
                                {
                                    // Check if data has a time
                                    if (!obs->insTime.empty())
                                    {
                                        node->pollEvents.insert(std::make_pair(obs->insTime, outputPin));
                                        break;
                                    }

                                    // Remove data without calling the callback if no time stamp
                                    // For post processing all data needs a time stamp
                                    node->callbacksEnabled = false;
                                    (node->**callback)(false);
                                    node->callbacksEnabled = true;
                                }
                                else
                                {
                                    outputPin->mode = OutputPin::Mode::REAL_TIME;
                                    for (auto& link : outputPin->links)
                                    {
                                        link.connectedNode->wakeWorker();
                                    }
                                    break;
                                }
                            }
                        }
                        else
                        {
                            LOG_ERROR("{} - {}: Callback is not valid anymore", node->nameId(), size_t(outputPin->id));
                        }

                        node->pollEvents.erase(it);
                    }

                    if (node->pollEvents.empty())
                    {
                        LOG_TRACE("{}: Finished polling all pins.", node->nameId());

                        node->callbacksEnabled = false;
                        for (auto& outputPin : node->outputPins)
                        {
                            if (outputPin.mode != OutputPin::Mode::REAL_TIME)
                            {
                                outputPin.mode = OutputPin::Mode::REAL_TIME;
                                for (auto& link : outputPin.links)
                                {
                                    link.connectedNode->wakeWorker();
                                }
                            }
                        }
                        node->_mode = Node::Mode::REAL_TIME;
                        FlowExecutor::deregisterNode(node);
                    }
                }
            }

            // Check if node finished
            if (node->_mode == Mode::POST_PROCESSING)
            {
                if (std::all_of(node->inputPins.begin(), node->inputPins.end(), [](const InputPin& inputPin) {
                        return inputPin.type != Pin::Type::Flow || !inputPin.isPinLinked() || inputPin.link.connectedNode->isDisabled()
                               || (inputPin.queue.empty() && inputPin.link.getConnectedPin()->mode == OutputPin::Mode::REAL_TIME);
                    }))
                {
                    node->callbacksEnabled = false;
                    for (auto& outputPin : node->outputPins)
                    {
                        outputPin.mode = OutputPin::Mode::REAL_TIME;
                        for (auto& link : outputPin.links)
                        {
                            link.connectedNode->wakeWorker();
                        }
                    }
                    node->_mode = Node::Mode::REAL_TIME;
                    FlowExecutor::deregisterNode(node);
                }
            }
        }
    }

    LOG_TRACE("{}: Worker thread ended.", node->nameId());
}

bool NAV::Node::workerInitializeNode()
{
    LOG_TRACE("{}: called", nameId());

    INS_ASSERT_USER_ERROR(_state == State::DoInitialize, fmt::format("Worker can only initialize the node if the state is set to DoInitialize, but it is {}.", toString(_state)).c_str());
    _state = Node::State::Initializing;
    _mode = Node::Mode::REAL_TIME;

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
    LOG_TRACE("{}: calling initialize()", nameId());
    if (initialize())
    {
        LOG_TRACE("{}: initialize() was successful", nameId());

        for (auto& inputPin : inputPins)
        {
            inputPin.queue.clear();
            inputPin.queueBlocked = false;
        }

        pollEvents.clear();
        LOG_TRACE("{}: calling resetNode()", nameId());
        resetNode();
        LOG_TRACE("{}: resetNode() was successful", nameId());
        for (auto& outputPin : outputPins)
        {
            outputPin.mode = OutputPin::Mode::REAL_TIME;
            for (auto& link : outputPin.links)
            {
                LOG_TRACE("{}: Waking connected node '{}'", nameId(), link.connectedNode->nameId());
                link.connectedNode->wakeWorker();
            }
        }

        if (_state == State::Initializing)
        {
            _state = Node::State::Initialized;
        }
        return true;
    }

    LOG_TRACE("{}: initialize() failed", nameId());
    if (_state == State::Initializing)
    {
        _state = Node::State::Deinitialized;
    }
    return false;
}

bool NAV::Node::workerDeinitializeNode()
{
    LOG_TRACE("{}: called", nameId());

    INS_ASSERT_USER_ERROR(_state == State::DoDeinitialize, fmt::format("Worker can only deinitialize the node if the state is set to DoDeinitialize, but it is {}.", toString(_state)).c_str());
    {
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
            _state = State::Disabled;
        }
        else if (_reinitialize)
        {
            _state = State::DoInitialize;
        }
        else
        {
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
