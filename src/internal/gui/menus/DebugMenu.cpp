#include "DebugMenu.hpp"

#include <imgui.h>

#include "internal/gui/NodeEditorApplication.hpp"

void NAV::gui::menus::ShowDebugMenu()
{
    ImGui::MenuItem("Show ImGui Demo Window", nullptr, &NodeEditorApplication::showImGuiDemoWindow);
    ImGui::MenuItem("Show ImPlot Demo Window", nullptr, &NodeEditorApplication::showImPlotDemoWindow);
}