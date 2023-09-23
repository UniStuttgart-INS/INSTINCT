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

#include "ColormapEditor.hpp"

#include <imgui.h>
#include <imgui_stdlib.h>
#include <fmt/format.h>
#include <vector>

#include "util/Plot/Colormap.hpp"
#include "internal/FlowManager.hpp"

void NAV::gui::windows::ShowColormapEditor(bool* show)
{
    if (!ImGui::Begin("Colormap Editor", show))
    {
        ImGui::End();
        return;
    }

    auto showColormaps = [](std::vector<Colormap>& colormaps, bool flow) {
        int colormapRemovalIdx = -1;

        if (ImGui::BeginTable("##{} colormap table", 4, ImGuiTableFlags_SizingFixedFit, ImVec2(0.0F, 0.0F)))
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
                if (ImGui::InputText(fmt::format("##colormap name {}", i).c_str(), &cm.name) && flow)
                {
                    flow::ApplyChanges();
                }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(145.0F);
                if (ImGui::Checkbox(fmt::format("##colormap discrete {}", i).c_str(), &cm.discrete) && flow)
                {
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Discrete?"); }

                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                if (ColormapButton(fmt::format("##colormap button {}", i).c_str(), cm, ImVec2(-1.0F, 0.0F)) && flow)
                {
                    flow::ApplyChanges();
                }

                ImGui::TableNextColumn();
                if (ImGui::Button(fmt::format("X##remove colormap {}", i).c_str())) { colormapRemovalIdx = static_cast<int>(i); }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Remove?"); }
            }

            ImGui::EndTable();
        }

        if (colormapRemovalIdx >= 0)
        {
            colormaps.erase(colormaps.begin() + static_cast<std::ptrdiff_t>(colormapRemovalIdx));
            if (flow) { flow::ApplyChanges(); }
        }

        if (ImGui::Button("Add##colormap"))
        {
            colormaps.emplace_back();
            if (flow) { flow::ApplyChanges(); }
        }
    };

    if (ImGui::BeginTabBar("Colormap TabBar"))
    {
        if (ImGui::BeginTabItem("Global"))
        {
            showColormaps(ColormapsGlobal, false);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Flow"))
        {
            showColormaps(ColormapsFlow, true);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}
