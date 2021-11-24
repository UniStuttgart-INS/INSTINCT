#include "TimeEdit.hpp"

#include <imgui.h>
#include <fmt/core.h>

#include "internal/gui/widgets/imgui_ex.hpp"

bool NAV::gui::widgets::TimeEdit(const char* str_id, InsTime& insTime, TimeEditFormat& timeEditFormat, float itemWidth)
{
    bool changes = false;

    ImGui::BeginGroup();

    if (ImGui::RadioButton(fmt::format("YMDHMS##{}", str_id).c_str(), reinterpret_cast<int*>(&timeEditFormat), static_cast<int>(TimeEditFormat::YMDHMS)))
    {
        changes = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton(fmt::format("GPS Week/ToW##{}", str_id).c_str(), reinterpret_cast<int*>(&timeEditFormat), static_cast<int>(TimeEditFormat::GPSWeekToW)))
    {
        changes = true;
    }

    if (timeEditFormat == TimeEditFormat::YMDHMS)
    {
        auto ymdhms = insTime.toYMDHMS();

        int year = ymdhms.year;
        int month = ymdhms.month;
        int day = ymdhms.day;
        int hour = ymdhms.hour;
        int min = ymdhms.min;
        auto sec = static_cast<double>(ymdhms.sec);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Year##{}", str_id).c_str(), &year, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Month##{}", str_id).c_str(), &month, 1, InsTimeUtil::MONTHS_PER_YEAR, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Day##{}", str_id).c_str(), &day, 1, InsTimeUtil::daysInMonth(month, year), 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Hour##{}", str_id).c_str(), &hour, 0, InsTimeUtil::HOURS_PER_DAY - 1, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Min##{}", str_id).c_str(), &min, 0, InsTimeUtil::MINUTES_PER_HOUR - 1, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("Sec##{}", str_id).c_str(), &sec, 0, InsTimeUtil::SECONDS_PER_MINUTE - 1, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }

        if (changes)
        {
            insTime = InsTime{ static_cast<uint16_t>(year), static_cast<uint16_t>(month), static_cast<uint16_t>(day),
                               static_cast<uint16_t>(hour), static_cast<uint16_t>(min), sec };
        }
    }
    else // if (timeEditFormat == TimeEditFormat::GPSWeekToW)
    {
        auto gpsWeekTow = insTime.toGPSweekTow();

        int cycle = gpsWeekTow.gpsCycle;
        int week = gpsWeekTow.gpsWeek;
        auto tow = static_cast<double>(gpsWeekTow.tow);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Cycle##{}", str_id).c_str(), &cycle, 0, std::numeric_limits<int>::max(), 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputIntL(fmt::format("Week##{}", str_id).c_str(), &week, 0, InsTimeUtil::WEEKS_PER_GPS_CYCLE - 1, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDoubleL(fmt::format("ToW [s]##{}", str_id).c_str(), &tow, 0, std::numeric_limits<double>::max(), 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }

        if (changes)
        {
            insTime = InsTime{ cycle, week, tow };
        }
    }

    ImGui::EndGroup();

    return changes;
}