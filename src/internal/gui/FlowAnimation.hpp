// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file FlowAnimation.hpp
/// @brief Handles Flow animations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-12-13

#pragma once

#include <imgui_node_editor.h>

namespace NAV::FlowAnimation
{

/// @brief Thread safe flow animaton of links
/// @param[in] id Id of the link to animate the flow with
/// @param[in] direction The direction to show the flow
void Add(ax::NodeEditor::LinkId id, ax::NodeEditor::FlowDirection direction = ax::NodeEditor::FlowDirection::Forward);

/// @brief Triggers the queued flow animations
void ProcessQueue();

} // namespace NAV::FlowAnimation
