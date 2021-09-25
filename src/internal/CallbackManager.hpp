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
void registerNodeCallbackForInvocation(const NodeCallback& nodeCallback, const std::shared_ptr<const NAV::NodeData>&);

void registerNotifyFunctionForInvocation(const NotifyFunction& notifyFunction);

void registerWatcherCallbackForInvocation(const WatcherCallback& watcherCallback, const std::shared_ptr<const NAV::NodeData>&);

void processNextCallback();

bool hasUnprocessedCallbacks();

} // namespace CallbackManager
} // namespace NAV