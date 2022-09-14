#include "LeftPane.hpp"

#include <imgui_node_editor.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/gui/TouchTracker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "util/StringUtil.hpp"

#include <string>
#include <algorithm>

namespace ed = ax::NodeEditor;

bool NAV::gui::panels::ShowLeftPane(float paneWidth)
{
    auto& io = ImGui::GetIO();

    bool paneActive = false;

    float insLogoWidth = paneWidth < 150 ? paneWidth : 150;

    float insLogoHeight = insLogoWidth * 2967.0F / 4073.0F;
    float childHeight = ImGui::GetContentRegionAvail().y - insLogoHeight;

    ImGui::BeginGroup();

    ImGui::BeginChild("Selection", ImVec2(paneWidth, childHeight));

    paneWidth = ImGui::GetContentRegionAvailWidth();

    float colSum = ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x + ImGui::GetStyle().Colors[ImGuiCol_WindowBg].y + ImGui::GetStyle().Colors[ImGuiCol_WindowBg].z;
    ImTextureID instinctLogo = NodeEditorApplication::m_InstinctLogo.at(colSum > 2.0F ? 1 : 0);
    ImTextureID insLogo = NodeEditorApplication::m_InsLogo.at(colSum > 2.0F ? 1 : 0);

    ImGui::Image(instinctLogo, ImVec2(paneWidth, paneWidth * 3000.0F / 13070.0F));
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 5.0F);

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
        ImGui::Spring();
        ImGui::EndHorizontal();

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
        // TODO: The flow animations currently crash under windows
        ImGui::PushDisabled();
#endif

        ImGui::Checkbox("Show Callback Flow", &nm::showFlowWhenInvokingCallbacks);
        if (NodeEditorApplication::defaultFontRatio() == 1.F) { ImGui::SameLine(); }
        ImGui::Checkbox("Show Notify Flow", &nm::showFlowWhenNotifyingValueChange);

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
        ImGui::PopDisabled();
#endif
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
        for (const auto* node : nm::m_Nodes())
        {
            ImGui::PushID(node->id.AsPointer());
            auto start = ImGui::GetCursorScreenPos();

            if (const auto progress = gui::GetTouchProgress(node->id); progress != 0.0F)
            {
                ImGui::GetWindowDrawList()->AddLine(
                    start + ImVec2(-18, 0),
                    start + ImVec2(-18, ImGui::GetTextLineHeight()),
                    IM_COL32(255, 0, 0, 255 - static_cast<int>(255 * progress)), 4.0F);
            }

            // Circle to show init status
            ImU32 circleCol = 0;
            if (node->isDisabled())
            {
                circleCol = IM_COL32(192, 192, 192, 255);
            }
            else if (node->getState() == Node::State::DoInitialize)
            {
                circleCol = IM_COL32(144, 202, 238, 255);
            }
            else if (node->getState() == Node::State::Initializing)
            {
                circleCol = IM_COL32(143, 188, 143, 255);
            }
            else if (node->getState() == Node::State::DoDeinitialize)
            {
                circleCol = IM_COL32(255, 222, 122, 255);
            }
            else if (node->getState() == Node::State::Deinitializing)
            {
                circleCol = IM_COL32(240, 128, 128, 255);
            }
            else if (node->getState() == Node::State::Initialized)
            {
                circleCol = IM_COL32(0, 255, 0, 255);
            }
            else // if (!node->getState() == Node::State::Deinitialized)
            {
                circleCol = IM_COL32(255, 0, 0, 255);
            }
            ImGui::GetWindowDrawList()->AddCircleFilled(start + ImVec2(-8 + (NodeEditorApplication::defaultFontRatio() - 1.F) * 5.F, ImGui::GetTextLineHeight() / 2.0F + 1.0F),
                                                        5.0F * NodeEditorApplication::defaultFontRatio(), circleCol);

            bool isSelected = std::find(selectedNodes.begin(), selectedNodes.end(), node->id) != selectedNodes.end();
            if (NodeEditorApplication::defaultFontRatio() != 1.F) { ImGui::Indent((NodeEditorApplication::defaultFontRatio() - 1.F) * 15.F); }
            if (ImGui::Selectable((str::replaceAll_copy(node->name, "\n", "") + "##" + std::to_string(size_t(node->id))).c_str(), &isSelected))
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
            if (NodeEditorApplication::defaultFontRatio() != 1.F) { ImGui::Unindent((NodeEditorApplication::defaultFontRatio() - 1.F) * 15.F); }

            auto id = fmt::format("({})", size_t(node->id));
            auto textSize = ImGui::CalcTextSize(id.c_str(), nullptr);
            auto iconPanelPos = start + ImVec2(std::max(paneWidth, 150.F) - ImGui::GetStyle().FramePadding.x - ImGui::GetStyle().IndentSpacing - ImGui::GetStyle().ItemInnerSpacing.x * 1, ImGui::GetTextLineHeight() / 2);
            ImGui::GetWindowDrawList()->AddText(
                ImVec2(iconPanelPos.x - textSize.x - ImGui::GetStyle().ItemInnerSpacing.x, start.y),
                colSum > 2.0F ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 255, 255), id.c_str(), nullptr);

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
        if (ImGui::Button("Deselect"))
        {
            ed::ClearSelection();
        }
        ImGui::EndHorizontal();
        ImGui::Indent();
        for (size_t i = 0; i < nodeCount; ++i)
        {
            auto* node = nm::FindNode(selectedNodes[i]);
            ImGui::Text("%s (%lu)", node != nullptr ? node->name.c_str() : "", size_t(selectedNodes[i]));
        }
        for (size_t i = 0; i < linkCount; ++i)
        {
            ImGui::Text("Link (%lu)", size_t(selectedLinks[i]));
        }
        ImGui::Unindent();
    }

    if (ImGui::IsWindowFocused())
    {
        paneActive = true;
    }

    ImGui::EndChild();

    ImGui::Image(insLogo, ImVec2(insLogoWidth, insLogoHeight));

    ImGui::EndGroup();

    return paneActive;
}