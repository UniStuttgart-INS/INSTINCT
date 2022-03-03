/// @file CallbackManager.hpp
/// @brief Manages the order of the callbacks
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-09-25

#pragma once

#include <imgui_node_editor.h>
#include <memory>

#include "internal/Node/Pin.hpp"

namespace NAV
{
class Node;
class NodeData;

namespace CallbackManager
{
/// @brief Queue a callback to invoke with the specified data
/// @param[in] nodeCallback Callback to invoke
/// @param[in] data Data to pass to the callback
void queueNodeCallbackForInvocation(const NodeCallback& nodeCallback, const std::shared_ptr<const NAV::NodeData>& data);

/// @brief Queue a notify function to invoke
/// @param[in] notifyFunction Function to invoke
void queueNotifyFunctionForInvocation(const NotifyFunction& notifyFunction);

/// @brief Queue a watcher callback to invoke with the specified data
/// @param[in] watcherCallback Callback to invoke
/// @param[in] data Data to pass to the callback
void queueWatcherCallbackForInvocation(const WatcherCallback& watcherCallback, const std::shared_ptr<const NAV::NodeData>& data);

/// @brief Process the next callback in the queue
void processNextCallback();

/// @brief Checks if there are unprocessed callbacks left in the queue
/// @return True if the queue is not empty
bool hasUnprocessedCallbacks();

} // namespace CallbackManager
} // namespace NAV