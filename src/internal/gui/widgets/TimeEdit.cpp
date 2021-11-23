#include "TimeEdit.hpp"

#include <imgui.h>
#include <fmt/core.h>

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
        double sec = static_cast<double>(ymdhms.sec);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Year##{}", str_id).c_str(), &year, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Month##{}", str_id).c_str(), &month, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Day##{}", str_id).c_str(), &day, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Hour##{}", str_id).c_str(), &hour, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Min##{}", str_id).c_str(), &min, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("Sec##{}", str_id).c_str(), &sec, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
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
        double tow = static_cast<double>(gpsWeekTow.tow);

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Cycle##{}", str_id).c_str(), &cycle, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (cycle < 0)
            {
                cycle = 0;
            }
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputInt(fmt::format("Week##{}", str_id).c_str(), &week, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (week < 0)
            {
                week = 0;
            }
            if (week >= InsTimeUtil::WEEKS_PER_GPS_CYCLE)
            {
                cycle = 0;
            }
            changes = true;
        }
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble(fmt::format("ToW [s]##{}", str_id).c_str(), &tow, 0, 0, "%.6f", ImGuiInputTextFlags_EnterReturnsTrue))
        {
            if (tow < 0)
            {
                tow = 0;
            }
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