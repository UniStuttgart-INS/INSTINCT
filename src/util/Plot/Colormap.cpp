// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Colormap.cpp
/// @brief Colormap
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-22

#include "Colormap.hpp"

#include <algorithm>
#include <chrono>

namespace NAV
{

std::vector<Colormap> GlobalColormaps;

Colormap::Colormap()
{
    id = std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count();
}

void Colormap::addColor(double value, ImColor color)
{
    colormap.insert(std::upper_bound(colormap.begin(), colormap.end(), value,
                                     [](double value, const std::pair<double, ImColor>& item) { return value < item.first; }),
                    std::make_pair(value, color));
}

void Colormap::removeColor(size_t idx)
{
    if (idx >= colormap.size()) { return; }
    colormap.erase(colormap.begin() + static_cast<std::ptrdiff_t>(idx));
}

const std::vector<std::pair<double, ImColor>>& Colormap::getColormap() const
{
    return colormap;
}

void Colormap::render() const
{
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const float w = ImGui::CalcItemWidth();
    const float h = ImGui::GetFrameHeight();
    const ImRect bounds = ImRect(pos.x, pos.y, pos.x + w, pos.y + h);

    render(bounds);
}

void Colormap::render(const ImRect& bounds) const
{
    if (auto* drawList = ImGui::GetWindowDrawList())
    {
        const size_t n = colormap.size() >= 2 ? (discrete ? colormap.size() : colormap.size() - 1) : colormap.size();

        const float step = bounds.GetWidth() / static_cast<float>(n);
        ImRect rect(bounds.Min.x, bounds.Min.y, bounds.Min.x + step, bounds.Max.y);
        for (size_t i = 0; i < n; ++i)
        {
            ImU32 col1 = colormap.at(i).second;
            ImU32 col2 = discrete ? col1 : ImU32(colormap.at(i + 1).second);

            drawList->AddRectFilledMultiColor(rect.Min, rect.Max, col1, col2, col2, col1);
            rect.TranslateX(step);
        }
    }
}

bool ColormapButton(const char* label, const Colormap& cmap, const ImVec2& size_arg)
{
    ImGuiContext& G = *GImGui;
    const ImGuiStyle& style = G.Style;
    ImGuiWindow* Window = G.CurrentWindow;
    if (Window->SkipItems) { return false; }

    const ImVec2 pos = ImGui::GetCurrentWindow()->DC.CursorPos;
    const ImVec2 label_size = ImGui::CalcTextSize(label, nullptr, true);
    ImVec2 size = ImGui::CalcItemSize(size_arg, label_size.x + style.FramePadding.x * 2.0F, label_size.y + style.FramePadding.y * 2.0F);
    const ImRect rect = ImRect(pos.x, pos.y, pos.x + size.x, pos.y + size.y);
    cmap.render(rect);
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32_BLACK_TRANS);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1, 1, 1, 0.1F));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1, 1, 1, 0.2F));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar(1);
    return pressed;
}

} // namespace NAV
