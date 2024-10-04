// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PlotEventTooltip.hpp
/// @brief Tooltips for a plot events
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-15

#pragma once

#include <vector>
#include <string>

#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// Tooltip for plot events
struct PlotEventTooltip
{
    InsTime time;                   ///< Time of the event
    std::vector<std::string> texts; ///< List of event texts
};

} // namespace NAV