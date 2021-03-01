/// @file RunMenu.hpp
/// @brief Run Menu
/// @author T. Topp (thomas@topp.cc)
/// @date 2021-01-03

#pragma once

#include "Nodes/Node.hpp"

#include <deque>

namespace NAV::gui::menus
{
/// @brief Show the run menu dropdown
void ShowRunMenu(std::deque<std::pair<Node*, bool>>& initList);

} // namespace NAV::gui::menus
