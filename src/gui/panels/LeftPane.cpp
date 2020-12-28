#include "LeftPane.hpp"

#include <imgui_node_editor.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

#include "NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "gui/TouchTracker.hpp"
#include "gui/windows/StyleEditor.hpp"

#include <string>
#include <algorithm>

namespace ed = ax::NodeEditor;

void NAV::gui::panels::ShowLeftPane(float paneWidth)
{
    auto& io = ImGui::GetIO();

    ImGui::BeginChild("Selection", ImVec2(paneWidth, 0));

    paneWidth = ImGui::GetContentRegionAvailWidth();

    { // Control Buttons
        ImGui::BeginHorizontal("Control Buttons", ImVec2(paneWidth, 0));
        ImGui::Spring(0.0F, 0.0F);
        if (ImGui::Button("Zoom to Content"))
        {
            ed::NavigateToContent();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Shortcut: F");
        }
        ImGui::Spring(0.0F);
        if (ImGui::Button("Show Flow"))
        {
            for (const auto& link : nm::m_Links())
            {
                if (nm::FindPin(link.startPinId)->type == Pin::Type::Flow)
                {
                    ed::Flow(link.id);
                }
            }
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Shortcut: Z");
        }
        ImGui::Spring();
        static bool showStyleEditor = false;
        if (ImGui::Button("Edit Style"))
        {
            showStyleEditor = true;
        }
        ImGui::EndHorizontal();

        if (showStyleEditor)
        {
            gui::windows::ShowStyleEditor(&showStyleEditor);
        }
    }

    std::vector<ed::NodeId> selectedNodes;
    std::vector<ed::LinkId> selectedLinks;
    selectedNodes.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));
    selectedLinks.resize(static_cast<size_t>(ed::GetSelectedObjectCount()));

    auto nodeCount = static_cast<size_t>(ed::GetSelectedNodes(selectedNodes.data(), static_cast<int>(selectedNodes.size())));
    auto linkCount = static_cast<size_t>(ed::GetSelectedLinks(selectedLinks.data(), static_cast<int>(selectedLinks.size())));

    selectedNodes.resize(nodeCount);
    selectedLinks.resize(linkCount);

    { // Node List
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetCursorScreenPos(),
            ImGui::GetCursorScreenPos() + ImVec2(paneWidth, ImGui::GetTextLineHeight()),
            ImColor(ImGui::GetStyle().Colors[ImGuiCol_HeaderActive]), ImGui::GetTextLineHeight() * 0.25F);
        ImGui::Spacing();
        ImGui::SameLine();
        ImGui::TextUnformatted("Nodes");
        ImGui::Indent();
        for (const auto& node : nm::m_Nodes())
        {
            ImGui::PushID(node->id.AsPointer());
            auto start = ImGui::GetCursorScreenPos();

            if (const auto progress = gui::GetTouchProgress(node->id); progress != 0.0F)
            {
                ImGui::GetWindowDrawList()->AddLine(
                    start + ImVec2(-8, 0),
                    start + ImVec2(-8, ImGui::GetTextLineHeight()),
                    IM_COL32(255, 0, 0, 255 - static_cast<int>(255 * progress)), 4.0F);
            }

            bool isSelected = std::find(selectedNodes.begin(), selectedNodes.end(), node->id) != selectedNodes.end();
            if (ImGui::Selectable((node->name + "##" + std::to_string(reinterpret_cast<uintptr_t>(node->id.AsPointer()))).c_str(), &isSelected))
            {
                if (io.KeyCtrl)
                {
                    if (isSelected)
                    {
                        ed::SelectNode(node->id, true);
                    }
                    else
                    {
                        ed::DeselectNode(node->id);
                    }
                }
                else
                {
                    ed::SelectNode(node->id, false);
                }

                ed::NavigateToSelection();
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Type: %s", node->type().c_str());
            }

            auto id = std::string("(") + std::to_string(reinterpret_cast<uintptr_t>(node->id.AsPointer())) + ")";
            auto textSize = ImGui::CalcTextSize(id.c_str(), nullptr);
            auto iconPanelPos = start + ImVec2(paneWidth - ImGui::GetStyle().FramePadding.x - ImGui::GetStyle().IndentSpacing - ImGui::GetStyle().ItemInnerSpacing.x * 1, ImGui::GetTextLineHeight() / 2);
            ImGui::GetWindowDrawList()->AddText(
                ImVec2(iconPanelPos.x - textSize.x - ImGui::GetStyle().ItemInnerSpacing.x, start.y),
                IM_COL32(255, 255, 255, 255), id.c_str(), nullptr);

            ImGui::PopID();
        }
        ImGui::Unindent();
    }

    { // Selection List
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetCursorScreenPos(),
            ImGui::GetCursorScreenPos() + ImVec2(paneWidth, ImGui::GetTextLineHeight()),
            ImColor(ImGui::GetStyle().Colors[ImGuiCol_HeaderActive]), ImGui::GetTextLineHeight() * 0.25F);
        ImGui::Spacing();
        ImGui::SameLine();
        ImGui::TextUnformatted("Selection");

        ImGui::BeginHorizontal("Selection Stats", ImVec2(paneWidth, 0));
        size_t selectedCount = selectedNodes.size() + selectedLinks.size();
        ImGui::Text("%lu item%s selected", selectedCount, (selectedCount > 1 || selectedCount == 0) ? "s" : "");
        ImGui::Spring();
        if (ImGui::Button("Deselect All"))
        {
            ed::ClearSelection();
        }
        ImGui::EndHorizontal();
        ImGui::Indent();
        for (size_t i = 0; i < nodeCount; ++i) { ImGui::Text("%s (%lu)", nm::FindNode(selectedNodes[i])->name.c_str(), reinterpret_cast<uintptr_t>(selectedNodes[i].AsPointer())); }
        for (size_t i = 0; i < linkCount; ++i) { ImGui::Text("Link (%lu)", reinterpret_cast<uintptr_t>(selectedLinks[i].AsPointer())); }
        ImGui::Unindent();
    }

    ImGui::EndChild();
}