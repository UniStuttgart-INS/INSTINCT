#include "FileMenu.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

#include "NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "FlowManager.hpp"

#include <iostream>

void NAV::gui::menus::ShowFileMenu(GlobalActions& globalAction)
{
    if (ImGui::MenuItem("New Flow", "Ctrl+N"))
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
    if (ImGui::MenuItem("Open Flow", "Ctrl+O"))
    {
        globalAction = GlobalActions::Load;
    }
    if (ImGui::BeginMenu("Open Recent", false))
    {
        ImGui::MenuItem("fish_hat.c");
        ImGui::MenuItem("fish_hat.inl");
        ImGui::MenuItem("fish_hat.h");
        ImGui::MenuItem("fish_hat.h");
        // ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 5);
        ImGui::Separator();
        ImGui::MenuItem("More...");
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Save", "Ctrl+S"))
    {
        flow::SaveFlow(globalAction);
    }
    if (ImGui::MenuItem("Save As..", "Ctrl+Shift+S"))
    {
        globalAction = GlobalActions::SaveAs;
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Quit", "Ctrl+Q"))
    {
        globalAction = GlobalActions::Quit;
    }
}