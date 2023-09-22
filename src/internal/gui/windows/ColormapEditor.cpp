// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ColormapEditor.cpp
/// @brief Colormap editor window
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-22

#include <imgui.h>
#include <imgui_stdlib.h>
#include <fmt/format.h>
#include "ColormapEditor.hpp"

void NAV::gui::windows::ShowColormapEditor(bool* show, std::vector<Colormap>& colormaps, size_t id)
{
    if (!ImGui::Begin("Colormap Editor", show))
    {
        ImGui::End();
        return;
    }

    int colormapRemovalIdx = -1;

    if (ImGui::BeginTable(fmt::format("##{} colormap table", id).c_str(), 4, ImGuiTableFlags_SizingFixedFit, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthFixed, 150.0F);
        ImGui::TableSetupColumn("Discrete", ImGuiTableColumnFlags_WidthFixed, 25.0F);
        ImGui::TableSetupColumn("Map", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Delete", ImGuiTableColumnFlags_WidthFixed, 20.0F);

        for (size_t i = 0; i < colormaps.size(); i++)
        {
            auto& cm = colormaps.at(i);

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(152.0F);
            ImGui::InputText(fmt::format("##{} colormap name {}", id, i).c_str(), &cm.name);

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(145.0F);
            ImGui::Checkbox(fmt::format("##{} colormap discrete {}", id, i).c_str(), &cm.discrete);
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Discrete?"); }

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ColormapButton(fmt::format("##{} colormap button {}", id, i).c_str(), cm, ImVec2(-1.0F, 0.0F));

            ImGui::TableNextColumn();
            if (ImGui::Button(fmt::format("X##{} remove colormap {}", id, i).c_str())) { colormapRemovalIdx = static_cast<int>(i); }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Remove?"); }
        }

        ImGui::EndTable();
    }

    if (colormapRemovalIdx >= 0)
    {
        colormaps.erase(colormaps.begin() + static_cast<std::ptrdiff_t>(colormapRemovalIdx));
    }

    if (ImGui::Button(fmt::format("Add##{} add colormap", id).c_str()))
    {
        colormaps.emplace_back();
    }

    ImGui::End();
}
