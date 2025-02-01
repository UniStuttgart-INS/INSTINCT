// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NodeEditorStyleEditor.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>
#include "internal/gui/NodeEditorApplication.hpp"

namespace ed = ax::NodeEditor;

namespace NAV::gui::windows
{

bool nodeEditorLightMode = false;

} // namespace NAV::gui::windows

void NAV::gui::windows::ShowNodeEditorStyleEditor(bool* show, std::vector<ImVec4>& colors, const std::vector<const char*>& colorNames)
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
    ImGui::BeginHorizontal("Color Mode", ImVec2(paneWidth, 0), 1.0F);
    ImGui::TextUnformatted("Filter Colors");
    ImGui::Spring();
    if (ImGui::Checkbox("Light mode", &nodeEditorLightMode)) { ApplyDarkLightMode(colors); }
    ImGui::Spring(0);
    ImGui::RadioButton("RGB", &edit_mode, ImGuiColorEditFlags_DisplayRGB);
    ImGui::Spring(0);
    ImGui::RadioButton("HSV", &edit_mode, ImGuiColorEditFlags_DisplayHSV);
    ImGui::Spring(0);
    ImGui::RadioButton("HEX", &edit_mode, ImGuiColorEditFlags_DisplayHex);
    ImGui::EndHorizontal();

    static ImGuiTextFilter filter;
    filter.Draw("##NodeEditorStyleEditor", paneWidth);

    ImGui::Spacing();

    ImGui::PushItemWidth(-160);
    for (int i = 0; i < ed::StyleColor_Count; i++)
    {
        const auto* name = ed::GetStyleColorName(static_cast<ed::StyleColor>(i));
        if (!filter.PassFilter(name))
        {
            continue;
        }

        ImGui::ColorEdit4(name, &editorStyle.Colors[i].x, edit_mode); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    for (size_t i = 0; i < colors.size(); i++)
    {
        const auto* name = colorNames.at(i);
        if (!filter.PassFilter(name))
        {
            continue;
        }
        ImGui::ColorEdit4(name, &colors.at(i).x, edit_mode); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    ImGui::PopItemWidth();

    ImGui::End();
}

void NAV::gui::windows::ApplyDarkLightMode(std::vector<ImVec4>& colors)
{
    auto& editorStyle = ed::GetStyle();
    if (nodeEditorLightMode)
    {
        editorStyle.Colors[ed::StyleColor_Bg] = ImColor(255, 255, 255, 255);
        editorStyle.Colors[ed::StyleColor_Grid] = ImColor(120, 120, 120, 40);
        editorStyle.Colors[ed::StyleColor_GroupBg] = ImColor(130, 130, 130, 160);

        colors[NodeEditorApplication::COLOR_GROUP_HEADER_TEXT] = ImColor(0, 0, 0, 255);
        colors[NodeEditorApplication::COLOR_GROUP_HEADER_BG] = ImColor(115, 115, 115, 0);
        colors[NodeEditorApplication::COLOR_GROUP_OUTER_BORDER] = ImColor(255, 255, 255, 64);

        ImGui::StyleColorsLight();
        ImGui::GetStyle().Colors[ImGuiCol_WindowBg] = ImColor(255, 255, 255, 255);
        ImGui::GetStyle().Colors[ImGuiCol_FrameBg] = ImColor(240, 240, 240, 255);
    }
    else
    {
        ed::Style defaultStyle{};
        for (int i = 0; i < ed::StyleColor_Count; i++)
        {
            editorStyle.Colors[i] = defaultStyle.Colors[i]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        }
        ImGui::StyleColorsDark();
    }
}
