#include "DebugMenu.hpp"

#include <imgui.h>

#include "internal/gui/windows/Global.hpp"

void NAV::gui::menus::ShowDebugMenu()
{
    ImGui::MenuItem("Show ImGui Demo Window", nullptr, &gui::windows::showImGuiDemoWindow);
    ImGui::MenuItem("Show ImPlot Demo Window", nullptr, &gui::windows::showImPlotDemoWindow);
}