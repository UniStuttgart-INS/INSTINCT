#include "Shortcuts.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/FlowManager.hpp"

#include <iostream>

void NAV::gui::checkShortcuts(GlobalActions& globalAction)
{
    auto& io = ImGui::GetIO();
    // Available Flags for key modifiers
    // io.KeyCtrl;
    // io.KeyAlt;
    // io.KeyShift;
    // io.KeySuper;

    if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
    {
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_F)))
        {
            ed::NavigateToContent();
        }

        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Z)))
        {
            for (const auto& link : nm::m_Links())
            {
                if (nm::FindPin(link.startPinId)->type == Pin::Type::Flow)
                {
                    ed::Flow(link.id);
                }
            }
        }
    }
    else if (io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
    {
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_N)))
        {
            if (flow::HasUnsavedChanges())
            {
                globalAction = GlobalActions::Clear;
            }
            else
            {
                nm::DeleteAllLinks();
                nm::DeleteAllNodes();
                flow::DiscardChanges();
                flow::SetCurrentFilename("");
            }
        }
        else if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_S)))
        {
            flow::SaveFlow(globalAction);
        }
        else if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_O)))
        {
            globalAction = GlobalActions::Load;
        }
        else if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Q)))
        {
            globalAction = GlobalActions::Quit;
        }
    }
    else if (io.KeyCtrl && !io.KeyAlt && io.KeyShift && !io.KeySuper)
    {
        if (io.KeyShift && ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_S)))
        {
            globalAction = GlobalActions::SaveAs;
        }
    }
}