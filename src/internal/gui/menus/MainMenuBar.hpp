/// @file MainMenuBar.hpp
/// @brief Main Menu Bar
/// @author T. Topp (thomas@topp.cc)
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
