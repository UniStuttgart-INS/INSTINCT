// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "HelpMarker.hpp"

#include "imgui.h"
#include <imgui_internal.h>

void NAV::gui::widgets::HelpMarker(const char* desc, const char* symbol) // NOLINT(clang-diagnostic-unused-function)
{
    ImGui::TextDisabled("%s", symbol);

    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0F);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool NAV::gui::widgets::BeginHelpMarker(const char* symbol)
{
    ImGui::TextDisabled("%s", symbol);

    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0F);
        return true;
    }
    return false;
}

void NAV::gui::widgets::EndHelpMarker()
{
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}