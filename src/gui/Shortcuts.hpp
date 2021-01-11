/// @file Shortcuts.hpp
/// @brief Defines all available shortcuts
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-16

#pragma once

#include "gui/GlobalActions.hpp"

namespace NAV::gui
{
/// @brief Checks if a shortcut was pressed
void checkShortcuts(GlobalActions& globalAction);

} // namespace NAV::gui
