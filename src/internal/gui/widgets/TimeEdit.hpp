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
#include "Navigation/Time/TimeSystem.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
namespace gui::widgets
{

/// @brief Time Edit format and system
struct TimeEditFormat
{
    /// Format to edit the time in
    enum class Format
    {
        YMDHMS,     ///< YearMonthDayHourMinuteSecond (UTC)
        GPSWeekToW, ///< GPS Week and TimeOfWeek
        COUNT,      ///< Amount of items in the enum
    };

    Format format = Format::YMDHMS; ///< Time format
    TimeSystem system = GPST;       ///< Time System
};

/// @brief Inputs to edit an InsTime object
/// @param[in] str_id Unique id for the ImGui elements
/// @param[in, out] insTime Time object to modify
/// @param[in, out] timeEditFormat Format to modify the time in
/// @param[in] itemWidth Width of the widget items
/// @return True if changes were made to the object
bool TimeEdit(const char* str_id, InsTime& insTime, TimeEditFormat& timeEditFormat, float itemWidth = 170.0F);

/// @brief Converts the provided Object into a json object
/// @param[out] j Return Json object
/// @param[in] timeEditFormat Object to convert
void to_json(json& j, const TimeEditFormat& timeEditFormat);
/// @brief Converts the provided json object
/// @param[in] j Json object with the time system
/// @param[out] timeEditFormat Object to return
void from_json(const json& j, TimeEditFormat& timeEditFormat);

} // namespace gui::widgets

/// @brief Converts the enum to a string
/// @param[in] timeEditFormat Enum value to convert into text
/// @return String representation of the enum
const char* to_string(gui::widgets::TimeEditFormat::Format timeEditFormat);

} // namespace NAV
