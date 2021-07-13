#include "TimeMenu.hpp"

#include <imgui.h>

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"

#include <sstream>

void NAV::gui::menus::ShowTimeMenu()
{
    InsTime currentTime = util::time::GetCurrentInsTime();
    if (currentTime.empty())
    {
        ImGui::Text("Time N/A");
    }
    else
    {
        std::stringstream stream;
        stream << currentTime.toYMDHMS();
        ImGui::Text("%s", stream.str().c_str());
    }

    static int timeSys = 0;
    ImGui::RadioButton("YMDHMS", &timeSys, 0);
    ImGui::SameLine();
    ImGui::RadioButton("GPS Week/ToW", &timeSys, 1);

    static int year = currentTime.toYMDHMS().year;
    static int month = currentTime.toYMDHMS().month;
    static int day = currentTime.toYMDHMS().day;
    static int hour = currentTime.toYMDHMS().hour;
    static int min = currentTime.toYMDHMS().min;
    static double sec = static_cast<double>(currentTime.toYMDHMS().sec);

    static int cycle = currentTime.toGPSweekTow().gpsCycle;
    static int week = currentTime.toGPSweekTow().gpsWeek;
    static double tow = static_cast<double>(currentTime.toGPSweekTow().tow);

    if (timeSys == 0)
    {
        ImGui::InputInt("Year", &year, 0, 0);
        ImGui::InputInt("Month", &month, 0, 0);
        ImGui::InputInt("Day", &day, 0, 0);
        ImGui::InputInt("Hour", &hour, 0, 0);
        ImGui::InputInt("Min", &min, 0, 0);
        ImGui::InputDouble("Sec", &sec, 0, 0);

        if (ImGui::Button("Update"))
        {
            InsTime newTime(static_cast<uint16_t>(year),
                            static_cast<uint16_t>(month),
                            static_cast<uint16_t>(day),
                            static_cast<uint16_t>(hour),
                            static_cast<uint16_t>(min),
                            sec);
            util::time::ClearCurrentTime();
            util::time::SetCurrentTime(newTime);

            cycle = newTime.toGPSweekTow().gpsCycle;
            week = newTime.toGPSweekTow().gpsWeek;
            tow = static_cast<double>(newTime.toGPSweekTow().tow);
        }
    }
    else
    {
        ImGui::InputInt("Cycle", &cycle, 0, 0);
        ImGui::InputInt("Week", &week, 0, 0);
        ImGui::InputDouble("ToW [s]", &tow, 0, 0);

        if (ImGui::Button("Update"))
        {
            InsTime newTime(static_cast<uint16_t>(cycle),
                            static_cast<uint16_t>(week),
                            tow);
            util::time::ClearCurrentTime();
            util::time::SetCurrentTime(newTime);

            year = newTime.toYMDHMS().year;
            month = newTime.toYMDHMS().month;
            day = newTime.toYMDHMS().day;
            hour = newTime.toYMDHMS().hour;
            min = newTime.toYMDHMS().min;
            sec = static_cast<double>(newTime.toYMDHMS().sec);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        util::time::ClearCurrentTime();

        currentTime = util::time::GetCurrentInsTime();
        year = currentTime.toYMDHMS().year;
        month = currentTime.toYMDHMS().month;
        day = currentTime.toYMDHMS().day;
        hour = currentTime.toYMDHMS().hour;
        min = currentTime.toYMDHMS().min;
        sec = static_cast<double>(currentTime.toYMDHMS().sec);

        cycle = currentTime.toGPSweekTow().gpsCycle;
        week = currentTime.toGPSweekTow().gpsWeek;
        tow = static_cast<double>(currentTime.toGPSweekTow().tow);
    }
}