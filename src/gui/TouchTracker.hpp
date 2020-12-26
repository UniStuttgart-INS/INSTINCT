/// @file TouchTracker.hpp
/// @brief Touch Event Tracker
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

namespace NAV::gui
{
/// @brief  Trigger a touch event on the specified node
/// @param[in] id Id of the node to trigger the event on
void TouchNode(ax::NodeEditor::NodeId id);

/// @brief Get the Touch Progress for the specified node
/// @param[in] id Id of the Node to check
/// @return The Touch progress towards the touch time
float GetTouchProgress(ax::NodeEditor::NodeId id);

/// @brief Updates the touch events for all nodes
/// @param[in] deltaTime Time elapsed since last frame, in [seconds]
void UpdateTouch(float deltaTime);

} // namespace NAV::gui
