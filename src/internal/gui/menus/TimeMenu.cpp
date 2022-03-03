#include "TimeMenu.hpp"

#include <imgui.h>

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"
#include "internal/gui/widgets/TimeEdit.hpp"

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

    static gui::widgets::TimeEditFormat timeEditFormat = gui::widgets::TimeEditFormat::YMDHMS;

    if (gui::widgets::TimeEdit("TimeEditGlobalTime", currentTime, timeEditFormat))
    {
        if (currentTime != util::time::GetCurrentInsTime())
        {
            util::time::ClearCurrentTime();
            util::time::SetCurrentTime(currentTime);
        }
    }

    if (ImGui::Button("Reset"))
    {
        util::time::ClearCurrentTime();
    }
}