// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeEdit.hpp"

#include <imgui.h>
#include <fmt/core.h>

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"

namespace NAV
{

const char* to_string(gui::widgets::TimeEditFormat::Format timeEditFormat)
{
    switch (timeEditFormat)
    {
    case gui::widgets::TimeEditFormat::Format::YMDHMS:
        return "YMDHMS";
    case gui::widgets::TimeEditFormat::Format::GPSWeekToW:
        return "GPS Week/ToW";
    case gui::widgets::TimeEditFormat::Format::COUNT:
        break;
    }
    return "";
}

namespace gui::widgets
{

/// @brief Shows a ComboBox to select the time edit format
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] format Reference to the format to select
bool ComboTimeEditFormat(const char* label, TimeEditFormat::Format& format)
{
    return gui::widgets::EnumCombo(label, format);
}

} // namespace gui::widgets

} // namespace NAV

bool NAV::gui::widgets::TimeEdit(const char* str_id, InsTime& insTime, TimeEditFormat& timeEditFormat, float itemWidth)
{
    bool changes = false;
    bool edited = false;

    ImGui::BeginGroup();

    ImGui::SetNextItemWidth(140.0F);
    if (ComboTimeEditFormat(fmt::format("##ComboTimeEditFormat {}", str_id).c_str(), timeEditFormat.format))
    {
        changes = true;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0F);
    if (ComboTimeSystem(fmt::format("##ComboTimeSystem {}", str_id).c_str(), timeEditFormat.system))
    {
        changes = true;
    }

    if (timeEditFormat.format == TimeEditFormat::Format::YMDHMS)
    {
        auto ymdhms = insTime.toYMDHMS(timeEditFormat.system);

        int year = ymdhms.year;
        int month = ymdhms.month;
        int day = ymdhms.day;
        int hour = ymdhms.hour;
        int min = ymdhms.min;
        auto sec = static_cast<double>(ymdhms.sec);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Year##{}", str_id).c_str(), &year, 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Month##{}", str_id).c_str(), &month, 1, InsTimeUtil::MONTHS_PER_YEAR, 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Day##{}", str_id).c_str(), &day, 1, InsTimeUtil::daysInMonth(month, year), 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Hour##{}", str_id).c_str(), &hour, 0, InsTimeUtil::HOURS_PER_DAY - 1, 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Min##{}", str_id).c_str(), &min, 0, InsTimeUtil::MINUTES_PER_HOUR - 1, 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("Sec##{}", str_id).c_str(), &sec, 0, InsTimeUtil::SECONDS_PER_MINUTE - 1, 0, 0, "%.6f")) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        if (changes || edited)
        {
            insTime = InsTime{ static_cast<uint16_t>(year), static_cast<uint16_t>(month), static_cast<uint16_t>(day),
                               static_cast<uint16_t>(hour), static_cast<uint16_t>(min), sec, timeEditFormat.system };
        }
    }
    else // if (timeEditFormat == TimeEditFormat::Format::GPSWeekToW)
    {
        auto gpsWeekTow = insTime.toGPSweekTow(timeEditFormat.system);

        int cycle = gpsWeekTow.gpsCycle;
        int week = gpsWeekTow.gpsWeek;
        auto tow = static_cast<double>(gpsWeekTow.tow);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Cycle##{}", str_id).c_str(), &cycle, 0, std::numeric_limits<int>::max(), 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Week##{}", str_id).c_str(), &week, 0, InsTimeUtil::WEEKS_PER_GPS_CYCLE - 1, 0, 0)) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("ToW [s]##{}", str_id).c_str(), &tow, 0, std::numeric_limits<double>::max(), 0, 0, "%.6f")) { edited = true; }
        if (ImGui::IsItemDeactivatedAfterEdit()) { changes = true; }

        if (changes || edited)
        {
            insTime = InsTime{ cycle, week, tow, timeEditFormat.system };
        }
    }

    ImGui::EndGroup();

    return changes;
}

void NAV::gui::widgets::to_json(json& j, const TimeEditFormat& timeEditFormat)
{
    j = json{
        { "format", timeEditFormat.format },
        { "system", timeEditFormat.system },
    };
}

void NAV::gui::widgets::from_json(const json& j, TimeEditFormat& timeEditFormat)
{
    j.at("format").get_to(timeEditFormat.format);
    j.at("system").get_to(timeEditFormat.system);
}