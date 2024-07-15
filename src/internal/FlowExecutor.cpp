// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FlowExecutor.hpp"

#include "Navigation/GNSS/Positioning/AntexReader.hpp"
#include "util/Logger.hpp"
#include "Navigation/Time/InsTime.hpp"

#include "internal/Node/Node.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/ConfigManager.hpp"
#include "util/Time/TimeBase.hpp"

#include <chrono>
#include <map>
#include <variant>
#include <memory>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#ifdef TESTING
    #include <catch2/catch_test_macros.hpp>
#endif

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

std::mutex _mutex;
std::condition_variable _cv;

enum class State
{
    Idle,
    Starting,
    Running,
    Stopping,
};
State _state = State::Idle;

std::thread _thd;
std::atomic<size_t> _activeNodes{ 0 };
std::chrono::time_point<std::chrono::steady_clock> _startTime;

/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::FlowExecutor
{

/// @brief Main task of the thread
void execute();

} // namespace NAV::FlowExecutor

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

bool NAV::FlowExecutor::isRunning() noexcept
{
    std::scoped_lock<std::mutex> lk(_mutex);
    return _state != State::Idle;
}

void NAV::FlowExecutor::start()
{
    LOG_TRACE("called");

    stop();

    {
        std::scoped_lock<std::mutex> lk(_mutex);
        _state = State::Starting;
    }

    _thd = std::thread(execute);
}

void NAV::FlowExecutor::stop()
{
    LOG_TRACE("called");

    if (isRunning())
    {
        {
            std::scoped_lock<std::mutex> lk(_mutex);
            if (_state == State::Running || _state == State::Starting)
            {
                _state = State::Stopping;
                _cv.notify_all();
            }
        }

        waitForFinish();
    }

    if (_thd.joinable()) { _thd.join(); }
}

void NAV::FlowExecutor::waitForFinish()
{
    LOG_TRACE("Waiting for finish of FlowExecutor...");
    {
        std::unique_lock lk(_mutex);
        _cv.wait(lk, [] { return _state == State::Idle; });
    }

    {
        std::scoped_lock<std::mutex> lk(_mutex);
        if (_thd.joinable()) { _thd.join(); }
    }
    LOG_TRACE("FlowExecutor finished.");
}

void NAV::FlowExecutor::deregisterNode([[maybe_unused]] const Node* node)
{
    LOG_DEBUG("Node {} finished.", node->nameId());
    _activeNodes--;

    if (_activeNodes == 0)
    {
        std::scoped_lock<std::mutex> lk(_mutex);
        _state = State::Stopping;
        _cv.notify_all();
    }
}

void NAV::FlowExecutor::execute()
{
    LOG_TRACE("called");

    AntexReader::Get().reset();

    for (Node* node : nm::m_Nodes())
    {
        for (auto& inputPin : node->inputPins)
        {
            inputPin.queue.clear();
            inputPin.queueBlocked = false;
        }
        node->pollEvents.clear();
    }

    if (!nm::InitializeAllNodes()) // This wakes the threads
    {
        std::scoped_lock<std::mutex> lk(_mutex);
        _state = State::Idle;
        _cv.notify_all();
        return;
    }

    LOG_INFO("=====================================================");
    bool realTimeMode = std::any_of(nm::m_Nodes().begin(), nm::m_Nodes().end(), [](const Node* node) {
        return node && !node->isDisabled() && node->_onlyRealTime;
    });
    LOG_INFO("Executing in {} mode", realTimeMode ? "real-time" : "post-processing");

    util::time::SetMode(realTimeMode ? util::time::Mode::REAL_TIME : util::time::Mode::POST_PROCESSING);
    _activeNodes = 0;

    for (Node* node : nm::m_Nodes())
    {
        if (node == nullptr || node->kind == Node::Kind::GroupBox || !node->isInitialized()) { continue; }

        {
            std::scoped_lock<std::mutex> lk(_mutex);
            if (_state != State::Starting) { break; }
        }

        node->_mode = realTimeMode ? Node::Mode::REAL_TIME : Node::Mode::POST_PROCESSING;
        if (!realTimeMode)
        {
            _activeNodes += 1;
            LOG_TRACE("Putting node '{}' into post-processing mode and adding to active nodes.", node->nameId());
        }
        {
            std::scoped_lock<std::mutex> guard(node->_configWindowMutex);
            node->resetNode();
        }
        for (size_t i = 0; i < node->outputPins.size(); i++) // for (auto& outputPin : node->outputPins)
        {
            auto& outputPin = node->outputPins[i];
            if (outputPin.type == Pin::Type::Flow && outputPin.isPinLinked())
            {
                outputPin.noMoreDataAvailable = false;
                LOG_TRACE("    Setting pin '{}' to hasDataAvailable", outputPin.name);
            }

            if (std::holds_alternative<OutputPin::PollDataFunc>(outputPin.data))
            {
                LOG_TRACE("    Adding pin '{}' to data poll event list.", outputPin.name);
                node->pollEvents.insert(std::make_pair(InsTime(), std::make_pair(&outputPin, i)));
            }
            else if (std::holds_alternative<OutputPin::PeekPollDataFunc>(outputPin.data))
            {
                if (auto* callback = std::get_if<OutputPin::PeekPollDataFunc>(&outputPin.data);
                    !outputPin.noMoreDataAvailable && callback != nullptr && *callback != nullptr
                    && std::any_of(outputPin.links.begin(), outputPin.links.end(), [](const OutputPin::OutgoingLink& link) {
                           return link.connectedNode->isInitialized();
                       }))
                {
                    if (auto obs = (node->**callback)(i, true)) // Peek the data
                    {
                        LOG_TRACE("    Adding pin '{}' to data poll event list with time {}.", outputPin.name, obs->insTime);
                        node->pollEvents.insert(std::make_pair(obs->insTime, std::make_pair(&outputPin, i)));
                    }
                }
            }
        }
    }

    {
        std::scoped_lock<std::mutex> lk(_mutex);
        if (_state == State::Starting)
        {
            nm::EnableAllCallbacks();
            _state = State::Running;
        }
    }

    _startTime = std::chrono::steady_clock::now();
    LOG_INFO("Execution started");

    bool anyNodeRunning = false;
    for (Node* node : nm::m_Nodes()) // Search for node pins with data callbacks
    {
        if (node != nullptr && node->kind != Node::Kind::GroupBox && node->isInitialized())
        {
            anyNodeRunning = true;
            LOG_DEBUG("Waking up node {}", node->nameId());
            node->wakeWorker();
        }
    }

    if (anyNodeRunning)
    {
        // Wait for the nodes to finish
        bool timeout = true;
        auto timeoutDuration = std::chrono::minutes(1);
        while (timeout)
        {
            std::unique_lock lk(_mutex);

            if (realTimeMode)
            {
                _cv.wait(lk, [] { return _state == State::Stopping; });
                break;
            }

            timeout = !_cv.wait_for(lk, timeoutDuration, [] { return _state == State::Stopping; });
            if (timeout && _activeNodes == 0)
            {
                LOG_ERROR("FlowExecutor had a timeout, but all nodes finished already.");
#ifdef TESTING
                FAIL("The FlowExecutor should not have a timeout when all nodes are finished already.");
#endif
                break;
            }
        }
    }

    // Deinitialize
    LOG_DEBUG("Stopping FlowExecutor...");
    nm::DisableAllCallbacks();
    nm::ClearAllNodeQueues();

    for (Node* node : nm::m_Nodes())
    {
        if (node == nullptr || node->kind == Node::Kind::GroupBox || !node->isInitialized()) { continue; }

        node->_mode = Node::Mode::REAL_TIME;
        for (auto& outputPin : node->outputPins)
        {
            outputPin.noMoreDataAvailable = true;
        }
        node->flush();
    }

    if (!ConfigManager::Get<bool>("nogui")
        || (!ConfigManager::Get<bool>("sigterm") && !ConfigManager::Get<size_t>("duration")))
    {
        auto finish = std::chrono::steady_clock::now();
        [[maybe_unused]] std::chrono::duration<double> elapsed = finish - _startTime;
        LOG_INFO("Elapsed time: {} s", elapsed.count());
    }

    _activeNodes = 0;
    LOG_TRACE("FlowExecutor deinitialized.");
    {
        std::scoped_lock<std::mutex> lk(_mutex);
        _state = State::Idle;
        _cv.notify_all();
    }

    LOG_TRACE("Execute thread finished.");
}
