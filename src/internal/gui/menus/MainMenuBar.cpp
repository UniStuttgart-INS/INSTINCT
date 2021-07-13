#include "MainMenuBar.hpp"

#include "FileMenu.hpp"
#include "EditMenu.hpp"
#include "RunMenu.hpp"
#include "TimeMenu.hpp"

#include "internal/FlowManager.hpp"

#include <imgui.h>
#include <fmt/core.h>

void NAV::gui::menus::ShowMainMenuBar(GlobalActions& globalAction, std::deque<std::pair<Node*, bool>>& initList)
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
            ShowRunMenu(initList);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Time"))
        {
            ShowTimeMenu();
            ImGui::EndMenu();
        }
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