// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TouchTracker.hpp
/// @brief Touch Event Tracker
/// @author T. Topp (topp@ins.uni-stuttgart.de)
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
