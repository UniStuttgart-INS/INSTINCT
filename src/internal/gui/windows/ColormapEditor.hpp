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

#include <vector>

#include "util/Plot/Colormap.hpp"

namespace NAV::gui::windows
{

/// @brief Shows a window for editing the user defined value colormaps
/// @param[in, out] show Flag which indicates whether the window is shown
/// @param[in, out] colormaps Colormaps to edit
/// @param[in] id Unique id (e.g. node id) to make the GUI unique
void ShowColormapEditor(bool* show, std::vector<Colormap>& colormaps, size_t id = 0);

} // namespace NAV::gui::windows