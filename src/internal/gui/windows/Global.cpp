#include "Global.hpp"

#include <imgui.h>
#include <implot.h>

#include "internal/gui/windows/NodeEditorStyleEditor.hpp"
#include "internal/gui/windows/ImPlotStyleEditor.hpp"

namespace NAV::gui::windows
{
bool showImGuiDemoWindow = false;
bool showImPlotDemoWindow = false;
bool showNodeEditorStyleEditor = false;
bool showImPlotStyleEditor = false;

} // namespace NAV::gui::windows

void NAV::gui::windows::renderGlobalWindows()
{
    if (showImGuiDemoWindow)
    {
        ImGui::ShowDemoWindow();
    }
    if (showImPlotDemoWindow)
    {
        ImPlot::ShowDemoWindow();
    }
    if (showNodeEditorStyleEditor)
    {
        gui::windows::ShowNodeEditorStyleEditor(&showNodeEditorStyleEditor);
    }
    if (showImPlotStyleEditor)
    {
        gui::windows::ShowImPlotStyleEditor(&showImPlotStyleEditor);
    }
}