// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Splitter.hpp"

#include <imgui.h>
#include <imgui_internal.h>

#include <string>

bool NAV::gui::widgets::Splitter(const char* str_id, bool split_vertically, float thickness,
                                 float* size1, float* size2,
                                 float min_size1, float min_size2,
                                 float splitter_long_axis_size /* = -1.0F */)
{
    const ImGuiContext& g = *GImGui;
    ImGuiWindow* window = g.CurrentWindow;
    ImGuiID id = window->GetID(("##Splitter" + std::string(str_id)).c_str());
    ImRect bb;
    bb.Min = window->DC.CursorPos + (split_vertically ? ImVec2(*size1, 0.0F) : ImVec2(0.0F, *size1));
    bb.Max = bb.Min + ImGui::CalcItemSize(split_vertically ? ImVec2(thickness, splitter_long_axis_size) : ImVec2(splitter_long_axis_size, thickness), 0.0F, 0.0F);
    return ImGui::SplitterBehavior(bb, id, split_vertically ? ImGuiAxis_X : ImGuiAxis_Y, size1, size2, min_size1, min_size2, 0.0F);
}