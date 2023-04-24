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

namespace NAV::gui::windows
{
/// @brief Shows a window for editing the style of the Node Editor
/// @param[in, out] show Flag which indicates whether the window is shown
void ShowNodeEditorStyleEditor(bool* show = nullptr);

} // namespace NAV::gui::windows
