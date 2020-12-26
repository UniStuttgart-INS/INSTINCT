#include "EditMenu.hpp"

#include <imgui.h>

void NAV::gui::menus::ShowEditMenu()
{
    if (ImGui::MenuItem("Undo", "CTRL+Z", false, false)) {}
    if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}
    ImGui::Separator();
    if (ImGui::MenuItem("Cut", "CTRL+X", false, false)) {}
    if (ImGui::MenuItem("Copy", "CTRL+C", false, false)) {}
    if (ImGui::MenuItem("Paste", "CTRL+V", false, false)) {}
}