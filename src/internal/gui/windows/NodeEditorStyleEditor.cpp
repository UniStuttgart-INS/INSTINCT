// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NodeEditorStyleEditor.hpp"

#include <imgui_node_editor.h>

namespace ed = ax::NodeEditor;

void NAV::gui::windows::ShowNodeEditorStyleEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("Node Editor Style", show))
    {
        ImGui::End();
        return;
    }

    auto paneWidth = ImGui::GetContentRegionAvail().x;

    auto& editorStyle = ed::GetStyle();
    ImGui::BeginHorizontal("Style buttons", ImVec2(paneWidth, 0), 1.0F);
    ImGui::TextUnformatted("Values");
    ImGui::Spring();
    if (ImGui::Button("Reset to defaults"))
    {
        editorStyle = ed::Style();
    }
    ImGui::EndHorizontal();
    ImGui::Spacing();
    ImGui::DragFloat4("Node Padding", &editorStyle.NodePadding.x, 0.1F, 0.0F, 40.0F);
    ImGui::DragFloat("Node Rounding", &editorStyle.NodeRounding, 0.1F, 0.0F, 40.0F);
    ImGui::DragFloat("Node Border Width", &editorStyle.NodeBorderWidth, 0.1F, 0.0F, 15.0F);
    ImGui::DragFloat("Hovered Node Border Width", &editorStyle.HoveredNodeBorderWidth, 0.1F, 0.0F, 15.0F);
    ImGui::DragFloat("Selected Node Border Width", &editorStyle.SelectedNodeBorderWidth, 0.1F, 0.0F, 15.0F);
    ImGui::DragFloat("Pin Rounding", &editorStyle.PinRounding, 0.1F, 0.0F, 40.0F);
    ImGui::DragFloat("Pin Border Width", &editorStyle.PinBorderWidth, 0.1F, 0.0F, 15.0F);
    ImGui::DragFloat("Link Strength", &editorStyle.LinkStrength, 1.0F, 0.0F, 500.0F);
    // ImVec2  SourceDirection;
    // ImVec2  TargetDirection;
    ImGui::DragFloat("Scroll Duration", &editorStyle.ScrollDuration, 0.001F, 0.0F, 2.0F);
    ImGui::DragFloat("Flow Marker Distance", &editorStyle.FlowMarkerDistance, 1.0F, 1.0F, 200.0F);
    ImGui::DragFloat("Flow Speed", &editorStyle.FlowSpeed, 1.0F, 1.0F, 2000.0F);
    ImGui::DragFloat("Flow Duration", &editorStyle.FlowDuration, 0.001F, 0.0F, 5.0F);
    // ImVec2  PivotAlignment;
    // ImVec2  PivotSize;
    // ImVec2  PivotScale;
    // float   PinCorners;
    // float   PinRadius;
    // float   PinArrowSize;
    // float   PinArrowWidth;
    ImGui::DragFloat("Group Rounding", &editorStyle.GroupRounding, 0.1F, 0.0F, 40.0F);
    ImGui::DragFloat("Group Border Width", &editorStyle.GroupBorderWidth, 0.1F, 0.0F, 15.0F);

    ImGui::Separator();

    static ImGuiColorEditFlags edit_mode = ImGuiColorEditFlags_DisplayRGB;
    ImGui::BeginHorizontal("Color Mode", ImVec2(paneWidth, 0), 1.0f);
    ImGui::TextUnformatted("Filter Colors");
    ImGui::Spring();
    ImGui::RadioButton("RGB", &edit_mode, ImGuiColorEditFlags_DisplayRGB);
    ImGui::Spring(0);
    ImGui::RadioButton("HSV", &edit_mode, ImGuiColorEditFlags_DisplayHSV);
    ImGui::Spring(0);
    ImGui::RadioButton("HEX", &edit_mode, ImGuiColorEditFlags_DisplayHex);
    ImGui::EndHorizontal();

    static ImGuiTextFilter filter;
    filter.Draw("", paneWidth);

    ImGui::Spacing();

    ImGui::PushItemWidth(-160);
    for (int i = 0; i < ed::StyleColor_Count; ++i)
    {
        const auto* name = ed::GetStyleColorName(static_cast<ed::StyleColor>(i));
        if (!filter.PassFilter(name))
        {
            continue;
        }

        ImGui::ColorEdit4(name, &editorStyle.Colors[i].x, edit_mode); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    ImGui::PopItemWidth();

    ImGui::End();
}