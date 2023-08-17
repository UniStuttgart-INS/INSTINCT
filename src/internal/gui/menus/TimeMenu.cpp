// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeMenu.hpp"

#include <imgui.h>

#include "util/Time/TimeBase.hpp"
#include "Navigation/Time/TimeSystem.hpp"
#include "internal/gui/widgets/TimeEdit.hpp"

#include <sstream>

void NAV::gui::menus::ShowTimeMenu()
{
    static gui::widgets::TimeEditFormat timeEditFormat;

    InsTime currentTime = util::time::GetCurrentInsTime();
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