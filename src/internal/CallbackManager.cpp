#include "CallbackManager.hpp"

#include <queue>
#include <variant>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/gui/FlowAnimation.hpp"

namespace NAV::CallbackManager
{
/// Callback List
std::queue<std::variant<std::pair<NodeCallback, std::shared_ptr<const NAV::NodeData>>,
                        NotifyFunction,
                        std::pair<WatcherCallback, std::shared_ptr<const NAV::NodeData>>>>
    callbacks;

void queueNodeCallbackForInvocation(const NodeCallback& nodeCallback, const std::shared_ptr<const NAV::NodeData>& data)
{
    callbacks.push(std::make_pair(nodeCallback, data));
}

void queueNotifyFunctionForInvocation(const NotifyFunction& notifyFunction)
{
    callbacks.push(notifyFunction);
}

void queueWatcherCallbackForInvocation(const WatcherCallback& watcherCallback, const std::shared_ptr<const NAV::NodeData>& data)
{
    callbacks.push(std::make_pair(watcherCallback, data));
}

void processNextCallback()
{
    if (callbacks.empty())
    {
        return;
    }

    auto nextCallback = callbacks.front();
    callbacks.pop();

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
    return !callbacks.empty();
}

} // namespace NAV::CallbackManager