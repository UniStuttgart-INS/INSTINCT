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
/// @brief Initializes all Nodes if they are not initialized yet
/// @return True if all nodes are initialized
bool initialize();

/// @brief Deinitialize all Nodes
void deinitialize();

/// @brief Main task of the thread
void execute();

} // namespace NAV::FlowExecutor

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

/*
  TODO:
    - Start needs to start a thread to return right away -> execute function
    - execute:
        - Initialize all nodes if not yet, and reset them
        - Trigger all FileReader nodes and put them into a std::set
            - Set a running flag
            - Wake the node worker thread
            - execute will return then
        - FileReader nodes read till max_queue_size reached, then sleep.
            - Connected nodes have to call onQueueHasSpace() which defaults to an empty function, but FileReader nodes can wake themselves up here
        - FileReaders finishing reading have to call FlowExecutor::deregisterNode() -> count down the _activeNodes variable and if == 0, then
            - Go over all nodes
                - Disable the firable condition, that all pins need data for temporal order (only active in POST_PROCESSING)
                - Trigger all node threads
    - deinitialize: call flush on each node giving DataLoggers the chance to flush their buffers.
*/

bool NAV::FlowExecutor::isRunning() noexcept
{
    return (_execute.load(std::memory_order_acquire) && _thd.joinable())
           || _activeNodes > 0;
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

    _execute.store(false, std::memory_order_release);
    if (_thd.joinable())
    {
        _thd.join();
    }

    // TODO: Stop all nodes
}

void NAV::FlowExecutor::waitForFinish()
{
    LOG_TRACE("called");

    // TODO: atomic_flag wait here
    // https://www.modernescpp.com/index.php/performancecomparison-of-condition-variables-and-atomics-in-c-20

    if (isRunning())
    {
        _thd.join();
    }
}

void NAV::FlowExecutor::deregisterNode()
{
    _activeNodes--;
}

bool NAV::FlowExecutor::initialize()
{
    LOG_TRACE("called");

    if (nm::InitializeAllNodes())
    {
        for (Node* node : nm::m_Nodes())
        {
            if (node->isInitialized())
            {
                node->resetNode();
                for (auto& inputPin : node->inputPins)
                {
                    inputPin.queue.clear();
                }
                node->_mode = Node::Mode::POST_PROCESSING;
            }
        }

        nm::EnableAllCallbacks();
        return true;
    }

    return false;
}

void NAV::FlowExecutor::deinitialize()
{
    LOG_TRACE("called");

    _execute.store(false, std::memory_order_release);

    if (!ConfigManager::Get<bool>("nogui"))
    {
        nm::DisableAllCallbacks();
    }

    for (Node* node : nm::m_Nodes())
    {
        node->flush();
        node->_mode = Node::Mode::REAL_TIME;
    }

    if (!ConfigManager::Get<bool>("nogui")
        || (!ConfigManager::Get<bool>("sigterm") && !ConfigManager::Get<size_t>("duration")))
    {
        auto finish = std::chrono::steady_clock::now();
        [[maybe_unused]] std::chrono::duration<double> elapsed = finish - _startTime;
        LOG_INFO("Elapsed time: {} s", elapsed.count());
    }
}

void NAV::FlowExecutor::execute()
{
    LOG_TRACE("called");

    if (!initialize())
    {
        _execute.store(false, std::memory_order_release);
        return;
    }

    LOG_INFO("Post-processing started");
    _startTime = std::chrono::steady_clock::now();

    std::multimap<NAV::InsTime, OutputPin*> events;

    for (Node* node : nm::m_Nodes()) // Search for node pins with data callbacks
    {
        if (node == nullptr || !node->isInitialized())
        {
            continue;
        }

        for (auto& outputPin : node->outputPins)
        {
            if (!_execute.load(std::memory_order_acquire))
            {
                deinitialize();
                return;
            }

            if (outputPin.type == Pin::Type::Flow
#ifndef TESTING
                && outputPin.isPinLinked()
#endif
            )
            {
                if (auto* callback = std::get_if<OutputPin::PollDataFunc>(&outputPin.data);
                    callback != nullptr && *callback != nullptr)
                {
                    LOG_DEBUG("Enabling post-processing mode for node '{}' on pin '{} ({})'", node->nameId(), outputPin.name, size_t(outputPin.id));
                    outputPin.hasPollDataRemaining = true;
                    node->postprocessingRunning = true;
                }
            }
        }
    }
    for (Node* node : nm::m_Nodes()) // Search for node pins with data callbacks
    {
        if (node != nullptr && node->isInitialized() && node->postprocessingRunning)
        {
            _activeNodes += 1;
            node->wakeWorker();
        }
    }
}
