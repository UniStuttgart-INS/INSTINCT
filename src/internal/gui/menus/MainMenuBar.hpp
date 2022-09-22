// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MainMenuBar.hpp
/// @brief Main Menu Bar
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-17

#pragma once

#include "internal/gui/GlobalActions.hpp"
#include "internal/Node/Node.hpp"

#include <deque>

namespace NAV::gui::menus
{
/// @brief Shows the main menu bar and moves down the cursor
/// @param[in, out] globalAction Global Action to perform
void ShowMainMenuBar(GlobalActions& globalAction);

} // namespace NAV::gui::menus
