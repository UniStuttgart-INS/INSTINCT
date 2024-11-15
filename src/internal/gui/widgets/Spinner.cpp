// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Spinner.hpp"

#include <imgui_internal.h>

#include <cmath>

void NAV::gui::widgets::Spinner(const char* label, const ImU32& color, float radius, float thickness)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
    {
        return;
    }

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID id = window->GetID(label);

    ImVec2 pos = window->DC.CursorPos;
    ImVec2 size((radius) * 2 + style.FramePadding.x, (radius + style.FramePadding.y) * 2);

    const ImRect bb(pos, ImVec2(pos.x + size.x, pos.y + size.y));
    ImGui::ItemSize(bb, style.FramePadding.y);
    if (!ImGui::ItemAdd(bb, id))
    {
        return;
    }

    // Render
    window->DrawList->PathClear();

    int num_segments = 30;
    int start = static_cast<int>(std::abs(std::sin(g.Time * 1.8) * (num_segments - 5)));

    const float a_min = IM_PI * 2.0F * static_cast<float>(start) / static_cast<float>(num_segments);
    const float a_max = IM_PI * 2.0F * static_cast<float>(num_segments - 3) / static_cast<float>(num_segments);

    const ImVec2 centre = ImVec2(pos.x + radius, pos.y + radius + style.FramePadding.y);

    for (int i = 0; i < num_segments; i++)
    {
        const float a = a_min + (static_cast<float>(i) / static_cast<float>(num_segments)) * (a_max - a_min);
        window->DrawList->PathLineTo(ImVec2(centre.x + ImCos(a + static_cast<float>(g.Time) * 8) * radius,
                                            centre.y + ImSin(a + static_cast<float>(g.Time) * 8) * radius));
    }

    window->DrawList->PathStroke(color, false, thickness);
}