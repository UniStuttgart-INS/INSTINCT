/// @file MainMenuBar.hpp
/// @brief Main Menu Bar
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-17

#pragma once

#include "internal/gui/GlobalActions.hpp"
#include "Nodes/Node.hpp"

#include <deque>

namespace NAV::gui::menus
{
/// @brief Shows the main menu bar and moves down the cursor
/// @param[in, out] globalAction Global Action to perform
/// @param[in, out] initList List of Nodes to initialize or deinitialize
void ShowMainMenuBar(GlobalActions& globalAction, std::deque<std::pair<Node*, bool>>& initList);

} // namespace NAV::gui::menus
