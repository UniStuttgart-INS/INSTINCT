/// @file RunMenu.hpp
/// @brief Run Menu
/// @author T. Topp (thomas@topp.cc)
/// @date 2021-01-03

#pragma once

#include "internal/Node/Node.hpp"

#include <deque>

namespace NAV::gui::menus
{
/// @brief Show the run menu dropdown
/// @param[in, out] initList List of nodes which should be asynchronously initialized
void ShowRunMenu(std::deque<std::pair<Node*, bool>>& initList);

} // namespace NAV::gui::menus
