// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeEdit.hpp
/// @brief Widget to modify time point values
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-11-23

#pragma once

#include "Navigation/Time/InsTime.hpp"

namespace NAV::gui::widgets
{
/// Format to edit the time in
enum class TimeEditFormat
{
    YMDHMS,     ///< YearMonthDayHourMinuteSecond (UTC)
    GPSWeekToW, ///< GPS Week and TimeOfWeek
};

/// @brief Inputs to edit an InsTime object
/// @param[in] str_id Unique id for the ImGui elements
/// @param[in, out] insTime Time object to modify
/// @param[in, out] timeEditFormat Format to modify the time in
/// @param[in] itemWidth Width of the widget items
/// @return True if changes were made to the object
bool TimeEdit(const char* str_id, InsTime& insTime, TimeEditFormat& timeEditFormat, float itemWidth = 170.0F);

} // namespace NAV::gui::widgets
