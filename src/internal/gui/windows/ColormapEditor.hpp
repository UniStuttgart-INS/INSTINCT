// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ColormapEditor.hpp
/// @brief Colormap editor window
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-22

#pragma once

namespace NAV::gui::windows
{

/// @brief Shows a window for editing the user defined value colormaps
/// @param[in, out] show Flag which indicates whether the window is shown
void ShowColormapEditor(bool* show);

} // namespace NAV::gui::windows