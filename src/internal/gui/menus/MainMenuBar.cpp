// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MainMenuBar.hpp"

#include "FileMenu.hpp"
#include "EditMenu.hpp"
#include "RunMenu.hpp"
#include "TimeMenu.hpp"
#include "DebugMenu.hpp"

#include "internal/FlowManager.hpp"

#include <imgui.h>
#include <fmt/core.h>

void NAV::gui::menus::ShowMainMenuBar(GlobalActions& globalAction)
{
    auto& io = ImGui::GetIO();

    auto cursorPosition = ImGui::GetCursorPos();
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            ShowFileMenu(globalAction);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit"))
        {
            ShowEditMenu();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Run"))
        {
            ShowRunMenu();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Time"))
        {
            ShowTimeMenu();
            ImGui::EndMenu();
        }
        // #ifndef NDEBUG
        if (ImGui::BeginMenu("Debug"))
        {
            ShowDebugMenu();
            ImGui::EndMenu();
        }
        // #endif
        // Move cursor to the right, as ImGui::Spring() is not working inside menu bars
        std::string text = fmt::format("FPS: {:.2f} ({:.2g}ms)", io.Framerate, io.Framerate != 0.0F ? 1000.0F / io.Framerate : 0.0F);
        float textPosX = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x
                         - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x;
        ImGui::SetCursorPosX(textPosX);
        ImGui::Text("%s", text.c_str());

        ImGui::EndMainMenuBar();
    }
    // Move cursor down, because menu bar does not take up space
    ImGui::SetCursorPos({ cursorPosition.x, cursorPosition.y + ImGui::GetTextLineHeight() + 5 });
}