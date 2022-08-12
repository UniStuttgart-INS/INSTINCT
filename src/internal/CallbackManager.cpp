#include "CallbackManager.hpp"

#include <queue>
#include <variant>
#include <mutex>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/gui/FlowAnimation.hpp"

namespace NAV::CallbackManager
{

/// Variant data type for callbacks
using CallbackType = std::variant<std::pair<NodeCallbackInfo, std::shared_ptr<const NAV::NodeData>>,
                                  NotifyFunctionInfo,
                                  std::pair<WatcherCallbackInfo, std::shared_ptr<const NAV::NodeData>>>;

/// Mutex to lock the buffer so that the GUI thread and the calculation threads don't cause a data race
std::mutex _mutex;

/// Callback List
std::queue<CallbackType> _callbacks;

void queueNodeCallbackForInvocation(const NodeCallbackInfo& nodeCallback, const std::shared_ptr<const NAV::NodeData>& data)
{
    std::scoped_lock<std::mutex> guard(_mutex);

    _callbacks.push(std::make_pair(nodeCallback, data));
}

void queueNotifyFunctionForInvocation(const NotifyFunctionInfo& notifyFunction)
{
    std::scoped_lock<std::mutex> guard(_mutex);

    _callbacks.push(notifyFunction);
}

void queueWatcherCallbackForInvocation(const WatcherCallbackInfo& watcherCallback, const std::shared_ptr<const NAV::NodeData>& data)
{
    std::scoped_lock<std::mutex> guard(_mutex);

    _callbacks.push(std::make_pair(watcherCallback, data));
}

void processNextCallback()
{
    CallbackType nextCallback;
    {
        std::scoped_lock<std::mutex> guard(_mutex);
        if (_callbacks.empty())
        {
            return;
        }

        nextCallback = _callbacks.front();
        _callbacks.pop();
    }

    if (nextCallback.index() == 0) // NodeCallback
    {
        const auto& [nodeCallback, data] = std::get<0>(nextCallback);
        const auto& [node, callback, linkId] = nodeCallback;

        if (nm::showFlowWhenInvokingCallbacks)
        {
            FlowAnimation::Add(linkId);
        }

        std::invoke(callback, node, data, linkId);
    }
    else if (nextCallback.index() == 1) // NotifyFunction
    {
        // TODO: Implement this
    }
    else if (nextCallback.index() == 2) // WatcherCallback
    {
        const auto& [watcherCallback, data] = std::get<2>(nextCallback);
        const auto& callback = watcherCallback.first;

        callback(data);
    }
}

bool hasUnprocessedCallbacks()
{
    std::scoped_lock<std::mutex> guard(_mutex);
    return !_callbacks.empty();
}

} // namespace NAV::CallbackManager