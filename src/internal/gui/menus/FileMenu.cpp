// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FileMenu.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/FlowManager.hpp"

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