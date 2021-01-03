#include "FlowExecutor.hpp"

#include "util/Logger.hpp"
#include "util/InsTime.hpp"

#include "Nodes/Node.hpp"
#include "NodeData/InsObs.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include <chrono>
#include <map>
#include <variant>
#include <memory>

NAV::FlowExecutor::~FlowExecutor()
{
    if (_execute.load(std::memory_order_acquire))
    {
        stop();
    };
}

bool NAV::FlowExecutor::isRunning() const noexcept
{
    return (_execute.load(std::memory_order_acquire) && _thd.joinable());
}

void NAV::FlowExecutor::start()
{
    if (_execute.load(std::memory_order_acquire))
    {
        stop();
    };
    _execute.store(true, std::memory_order_release);
    _thd = std::thread(&FlowExecutor::execute, this);
}

void NAV::FlowExecutor::stop()
{
    _execute.store(false, std::memory_order_release);
    if (_thd.joinable())
    {
        _thd.join();
    }
}

void NAV::FlowExecutor::waitForFinish()
{
    if (isRunning())
    {
        _thd.join();
    }
}

bool NAV::FlowExecutor::initialize()
{
    LOG_TRACE("called");

    bool hasUninitializedNodes = false;
    for (Node* node : nm::m_Nodes())
    {
        if (!node->isInitialized)
        {
            if (!node->initialize())
            {
                LOG_ERROR("Node {} fails to initialize. Please check the node configuration.", node->nameId());
                hasUninitializedNodes = true;
            }
        }
    }

    return !hasUninitializedNodes;
}

void NAV::FlowExecutor::deinitialize()
{
    LOG_TRACE("called");

    for (Node* node : nm::m_Nodes())
    {
        node->deinitialize();
    }
}

void NAV::FlowExecutor::execute()
{
    LOG_TRACE("called");

    if (!initialize())
    {
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    std::multimap<NAV::InsTime, Pin*> events;

    for (Node* node : nm::m_Nodes())
    {
        if (node == nullptr)
        {
            continue;
        }

        for (Pin& outputPin : node->outputPins)
        {
            if (outputPin.type == Pin::Type::Flow)
            {
                auto callback = std::get_if<std::shared_ptr<NAV::NodeData> (Node::*)(bool)>(&outputPin.data);
                if (callback != nullptr && *callback != nullptr)
                {
                    LOG_DEBUG("Searching node {} on output pin {} for data", node->nameId(), size_t(outputPin.id));
                    bool dataEventCreated = false;
                    while (true)
                    {
                        // Check if data available (peek = true)
                        if (auto obs = std::static_pointer_cast<NAV::InsObs>((node->**callback)(true)))
                        {
                            // Check if data has a time
                            if (obs->insTime.has_value())
                            {
                                events.insert(std::make_pair(obs->insTime.value(), &outputPin));
                                LOG_INFO("Taking Data from {} on output pin {} into account.", node->nameId(), size_t(outputPin.id));
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
    while (it = events.begin(), it != events.end())
    {
        Pin* pin = it->second;
        Node* node = pin->parentNode;
        auto callback = std::get_if<std::shared_ptr<NAV::NodeData> (Node::*)(bool)>(&pin->data);
        if (callback != nullptr && *callback != nullptr)
        {
            // Trigger the already peeked observation and invoke it's callbacks (peek = false)
            if ((node->**callback)(false) == nullptr)
            {
                LOG_ERROR("{}: Could not poll its observation despite being able to peek it.", node->nameId());
            }

            // Add next data event from the node
            while (true)
            {
                // Check if data available (peek = true)
                if (auto obs = std::static_pointer_cast<NAV::InsObs>((node->**callback)(true)))
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

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    LOG_INFO("Elapsed time: {} s", elapsed.count());

    deinitialize();
}
