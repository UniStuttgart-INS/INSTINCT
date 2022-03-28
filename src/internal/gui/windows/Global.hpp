/// @file Global.hpp
/// @brief Global windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-22

#pragma once

namespace NAV::gui::windows
{

/// @brief Flag whether the ImGui Demo window should be displayed
extern bool showImGuiDemoWindow;
/// @brief Flag whether the ImPlot Demo window should be displayed
extern bool showImPlotDemoWindow;

/// @brief Flag whether the NodeEditor style editor windows should be displayed
extern bool showNodeEditorStyleEditor;
/// @brief Flag whether the ImPlot style editor windows should be displayed
extern bool showImPlotStyleEditor;

/// @brief Called every frame to render global windows
void renderGlobalWindows();

} // namespace NAV::gui::windows