// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Global.hpp"

#include <imgui.h>
#include <implot.h>

#include "internal/gui/windows/NodeEditorStyleEditor.hpp"
#include "internal/gui/windows/ImPlotStyleEditor.hpp"
#include "internal/gui/windows/FontSizeEditor.hpp"
#include "internal/gui/windows/ColormapEditor.hpp"
#include "internal/gui/windows/Screenshotter.hpp"

namespace NAV::gui::windows
{
bool showImGuiDemoWindow = false;
bool showImPlotDemoWindow = false;
bool showNodeEditorStyleEditor = false;
bool showImPlotStyleEditor = false;
bool showFontSizeEditor = false;
bool showColormapEditor = false;
bool showScreenshotter = false;

} // namespace NAV::gui::windows

void NAV::gui::windows::renderGlobalWindows(std::vector<ImVec4>& colors, const std::vector<const char*>& colorNames)
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
        gui::windows::ShowNodeEditorStyleEditor(&showNodeEditorStyleEditor, colors, colorNames);
    }
    if (showImPlotStyleEditor)
    {
        gui::windows::ShowImPlotStyleEditor(&showImPlotStyleEditor);
    }
    if (showFontSizeEditor)
    {
        gui::windows::ShowFontSizeEditor(&showFontSizeEditor);
    }
    if (showColormapEditor)
    {
        gui::windows::ShowColormapEditor(&showColormapEditor);
    }
#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W
    if (showScreenshotter)
    {
        gui::windows::ShowScreenshotter(&showScreenshotter);
    }
#endif
}