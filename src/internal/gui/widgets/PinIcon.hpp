// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file HelpMarker.hpp
/// @brief Icon Widget
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-14

#pragma once

#include "imgui.h"
#include <cstdint>

namespace NAV::gui::widgets::PinIcon
{
enum class Type : uint8_t
{
    Flow,
    Circle,
    Square,
    Grid,
    RoundSquare,
    Diamond
};

/// @brief Draws an Icon for a Pin with the specified settings
/// @param[in] size Size of the Icons
/// @param[in] type Type of the Icon (Flow, Circle, Square, Grid, ...)
/// @param[in] filled Specifies if the icons should be filled
/// @param[in] color Color of the Icon
/// @param[in] innerColor Color to fill the Icon with
void Draw(const ImVec2& size, Type type, bool filled, const ImVec4& color = ImVec4(1, 1, 1, 1), const ImVec4& innerColor = ImVec4(0, 0, 0, 0));

} // namespace NAV::gui::widgets::PinIcon