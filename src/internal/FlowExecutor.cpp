#include "FlowExecutor.hpp"

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

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

std::atomic<bool> _execute{ false };
std::thread _thd;
std::atomic<size_t> _activeNodes{ 0 };
std::chrono::time_point<std::chrono::steady_clock> _startTime;

/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::FlowExecutor
{

/// @brief Deinitialize all Nodes
void deinitialize();

/// @brief Main task of the thread
void execute();

} // namespace NAV::FlowExecutor

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

bool NAV::FlowExecutor::isRunning() noexcept
{
    return _execute.load(std::memory_order_acquire);
}

void NAV::FlowExecutor::start()
{
    stop();

    LOG_TRACE("called");

    _execute.store(true, std::memory_order_release);
    _thd = std::thread(execute);
}

void NAV::FlowExecutor::stop()
{
    LOG_TRACE("called");

    if (_thd.joinable())
    {
        _thd.join();
    }

    if (isRunning())
    {
        deinitialize();

        waitForFinish();
    }
}

void NAV::FlowExecutor::waitForFinish()
{
    LOG_TRACE("called");

    if (_thd.joinable()) { _thd.join(); }

    LOG_TRACE("Waiting for finish of FlowExecutor...");
    _execute.wait(true);
    LOG_TRACE("FlowExecutor finished.");
}

void NAV::FlowExecutor::deregisterNode([[maybe_unused]] const Node* node)
{
    LOG_DEBUG("Node {} finished.", node->nameId());
    _activeNodes--;

    if (_activeNodes == 0)
    {
        deinitialize();
    }
}

void NAV::FlowExecutor::deinitialize()
{
    LOG_TRACE("called");

    nm::DisableAllCallbacks();

    for (Node* node : nm::m_Nodes())
    {
        if (node == nullptr || !node->isInitialized()) { continue; }

        node->_mode = Node::Mode::REAL_TIME;
        for (auto& outputPin : node->outputPins)
        {
            outputPin.mode = OutputPin::Mode::REAL_TIME;
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

    if (_thd.joinable()) { _thd.join(); }

    _activeNodes = 0;
    _execute.store(false, std::memory_order_release);
    _execute.notify_all();
}

void NAV::FlowExecutor::execute()
{
    LOG_TRACE("called");

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
        _execute.store(false, std::memory_order_release);
        return;
    }

    for (Node* node : nm::m_Nodes())
    {
        if (node == nullptr || !node->isInitialized()) { continue; }

        node->_mode = Node::Mode::POST_PROCESSING;
        _activeNodes += 1;
        node->resetNode();
        LOG_TRACE("Putting node '{}' into post-processing mode and adding to active nodes.", node->nameId());
        for (auto& outputPin : node->outputPins)
        {
            if (outputPin.type == Pin::Type::Flow
#ifndef TESTING
                && outputPin.isPinLinked()
#endif
            )
            {
                outputPin.mode = OutputPin::Mode::POST_PROCESSING;
                LOG_TRACE("    Putting pin '{}' into post-processing mode", outputPin.name);
            }

            if (auto* callback = std::get_if<OutputPin::PollDataFunc>(&outputPin.data);
                outputPin.mode == OutputPin::Mode::POST_PROCESSING && callback != nullptr && *callback != nullptr
                && std::any_of(outputPin.links.begin(), outputPin.links.end(), [](const OutputPin::OutgoingLink& link) {
                       return link.connectedNode->isInitialized();
                   }))
            {
                if (auto obs = (node->**callback)(true)) // Peek the data
                {
                    LOG_TRACE("    Adding pin '{}' to data poll event list with time {}.", outputPin.name, obs->insTime);
                    node->pollEvents.insert(std::make_pair(obs->insTime, &outputPin));
                }
            }
        }
    }

    nm::EnableAllCallbacks();

    LOG_INFO("Post-processing started");
    _startTime = std::chrono::steady_clock::now();

    for (Node* node : nm::m_Nodes()) // Search for node pins with data callbacks
    {
        if (node != nullptr && node->isInitialized())
        {
            LOG_DEBUG("Waking up node {}", node->nameId());
            node->wakeWorker();
        }
    }
}
