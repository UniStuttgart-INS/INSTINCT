// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Shortcuts.hpp
/// @brief Defines all available shortcuts
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-16

#pragma once

#include "internal/gui/GlobalActions.hpp"

namespace NAV::gui
{
/// @brief Checks if a shortcut was pressed
void checkShortcuts(GlobalActions& globalAction);

} // namespace NAV::gui
