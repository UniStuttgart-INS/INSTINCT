// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NodeEditorStyleEditor.hpp
/// @brief Style Editor window
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-14

#pragma once

#include <vector>
#include <imgui.h>

namespace NAV::gui::windows
{
/// @brief Shows a window for editing the style of the Node Editor
/// @param[in, out] show Flag which indicates whether the window is shown
/// @param[in, out] colors Colors of the GUI
/// @param[in] colorNames Names for the colors
void ShowNodeEditorStyleEditor(bool* show, std::vector<ImVec4>& colors, const std::vector<const char*>& colorNames);

/// Applies the currently selected mode
/// @param[in, out] colors Colors of the GUI
void ApplyDarkLightMode(std::vector<ImVec4>& colors);

/// If true, light mode is selected
extern bool nodeEditorLightMode;

} // namespace NAV::gui::windows
