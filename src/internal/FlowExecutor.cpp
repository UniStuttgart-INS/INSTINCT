#include "FlowExecutor.hpp"

#include "util/Logger.hpp"
#include "Navigation/Time/InsTime.hpp"

#include "internal/Node/Node.hpp"
#include "NodeData/InsObs.hpp"

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

bool NAV::FlowExecutor::isRunning() noexcept
{
    return (_execute.load(std::memory_order_acquire) && _thd.joinable());
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
}

void NAV::FlowExecutor::waitForFinish()
{
    LOG_TRACE("called");

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

    std::multimap<NAV::InsTime, Pin*> events;

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
                && nm::IsPinLinked(outputPin)
#endif
            )
            {
                auto* callback = std::get_if<std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>(&outputPin.dataOld);
                if (callback != nullptr && *callback != nullptr)
                {
                    LOG_DEBUG("Searching node {} on output pin {} (id {}) for data", node->nameId(), node->pinIndexFromId(outputPin.id), size_t(outputPin.id));
                    bool dataEventCreated = false;
                    while (true)
                    {
                        // Check if data available (peek = true)
                        if (auto obs = std::static_pointer_cast<const NAV::InsObs>((node->**callback)(true)))
                        {
                            // Check if data has a time
                            if (obs->insTime.has_value())
                            {
                                events.insert(std::make_pair(obs->insTime.value(), &outputPin));
                                LOG_DEBUG("Taking Data from {} on output pin {} into account.", node->nameId(), size_t(outputPin.id));
                                dataEventCreated = true;
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
                            break;
                        }
                    }
                    if (!dataEventCreated)
                    {
                        node->resetNode();
                    }
                }
            }
        }
    }

    LOG_INFO("Processing Data from files");
    std::multimap<NAV::InsTime, Pin*>::iterator it;
    while (it = events.begin(), it != events.end() && _execute.load(std::memory_order_acquire))
    {
        Pin* pin = it->second;
        Node* node = pin->parentNode;
        auto* callback = std::get_if<std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>(&pin->dataOld);
        if (callback != nullptr && *callback != nullptr)
        {
            // Update the global time
            util::time::SetCurrentTime(it->first);

            // Trigger the already peeked observation and invoke it's callbacks (peek = false)
            if ((node->**callback)(false) == nullptr)
            {
                LOG_ERROR("{}: Could not poll its observation despite being able to peek it.", node->nameId());
            }

            // Add next data event from the node
            while (true)
            {
                // Check if data available (peek = true)
                if (auto obs = std::static_pointer_cast<const NAV::InsObs>((node->**callback)(true)))
                {
                    // Check if data has a time
                    if (obs->insTime.has_value())
                    {
                        events.insert(std::make_pair(obs->insTime.value(), pin));
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
                    break;
                }
            }
        }
        else
        {
            LOG_ERROR("{} - {}: Callback is not valid anymore", node->nameId(), size_t(pin->id));
        }

        events.erase(it);
    }

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
