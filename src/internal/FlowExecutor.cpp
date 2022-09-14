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
            }
        }
        util::time::SetMode(util::time::Mode::POST_PROCESSING);
        util::time::ClearCurrentTime();

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
    }

    util::time::SetMode(util::time::Mode::REAL_TIME);
}

void NAV::FlowExecutor::execute()
{
    LOG_TRACE("called");

    if (!initialize())
    {
        _execute.store(false, std::memory_order_release);
        return;
    }

    auto start = std::chrono::steady_clock::now();

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
                // TODO: Refactor this
                // auto* callback = std::get_if<std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>(&outputPin.dataOld);
                // if (callback != nullptr && *callback != nullptr)
                // {
                //     LOG_DEBUG("Searching node {} on output pin {} (id {}) for data", node->nameId(), node->outputPinIndexFromId(outputPin.id), size_t(outputPin.id));
                //     bool dataEventCreated = false;
                //     while (true)
                //     {
                //         // Check if data available (peek = true)
                //         if (auto obs = std::static_pointer_cast<const NAV::InsObs>((node->**callback)(true)))
                //         {
                //             // Check if data has a time
                //             if (obs->insTime.has_value())
                //             {
                //                 events.insert(std::make_pair(obs->insTime.value(), &outputPin));
                //                 LOG_DEBUG("Taking Data from {} on output pin {} into account.", node->nameId(), size_t(outputPin.id));
                //                 dataEventCreated = true;
                //                 break;
                //             }

                //             // Remove data without calling the callback if no time stamp
                //             // For post processing all data needs a time stamp
                //             node->callbacksEnabled = false;
                //             (node->**callback)(false);
                //             node->callbacksEnabled = true;
                //         }
                //         else
                //         {
                //             break;
                //         }
                //     }
                //     if (!dataEventCreated)
                //     {
                //         node->resetNode();
                //     }
                // }
            }
        }
    }

    LOG_INFO("Processing Data from files");
    // TODO: Refactor this
    // std::multimap<NAV::InsTime, OutputPin*>::iterator it;
    // while (it = events.begin(), it != events.end() && _execute.load(std::memory_order_acquire))
    // {
    //     OutputPin* pin = it->second;
    //     Node* node = pin->parentNode;
    //     auto* callback = std::get_if<std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>(&pin->dataOld);
    //     if (callback != nullptr && *callback != nullptr)
    //     {
    //         // Update the global time
    //         util::time::SetCurrentTime(it->first);

    //         // Trigger the already peeked observation and invoke it's callbacks (peek = false)
    //         if ((node->**callback)(false) == nullptr)
    //         {
    //             LOG_ERROR("{}: Could not poll its observation despite being able to peek it.", node->nameId());
    //         }

    //         // Add next data event from the node
    //         while (true)
    //         {
    //             // Check if data available (peek = true)
    //             if (auto obs = std::static_pointer_cast<const NAV::InsObs>((node->**callback)(true)))
    //             {
    //                 // Check if data has a time
    //                 if (obs->insTime.has_value())
    //                 {
    //                     events.insert(std::make_pair(obs->insTime.value(), pin));
    //                     break;
    //                 }

    //                 // Remove data without calling the callback if no time stamp
    //                 // For post processing all data needs a time stamp
    //                 node->callbacksEnabled = false;
    //                 (node->**callback)(false);
    //                 node->callbacksEnabled = true;
    //             }
    //             else
    //             {
    //                 break;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         LOG_ERROR("{} - {}: Callback is not valid anymore", node->nameId(), size_t(pin->id));
    //     }

    //     events.erase(it);
    // }

    if (!ConfigManager::Get<bool>("nogui")
        || (!ConfigManager::Get<bool>("sigterm")
            && !ConfigManager::Get<size_t>("duration")))
    {
        auto finish = std::chrono::steady_clock::now();
        [[maybe_unused]] std::chrono::duration<double> elapsed = finish - start;
        LOG_INFO("Elapsed time: {} s", elapsed.count());
    }

    deinitialize();
}
