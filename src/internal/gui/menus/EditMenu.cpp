// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "EditMenu.hpp"

#include <imgui.h>
#include "internal/gui/GlobalActions.hpp"
#include "internal/gui/windows/Global.hpp"

void NAV::gui::menus::ShowEditMenu()
{
    if (ImGui::MenuItem("Undo", "CTRL+Z", false, canUndoLastAction()))
    {
        undoLastAction();
    }
    if (ImGui::MenuItem("Redo", "CTRL+Y", false, canRedoLastAction()))
    {
        redoLastAction();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Cut", "CTRL+X", false, canCutOrCopyFlowElements()))
    {
        cutFlowElements();
    }
    if (ImGui::MenuItem("Copy", "CTRL+C", false, canCutOrCopyFlowElements()))
    {
        copyFlowElements();
    }
    if (ImGui::MenuItem("Paste", "CTRL+V", false, canPasteFlowElements()))
    {
        pasteFlowElements();
    }
    ImGui::Separator();
    ImGui::MenuItem("Node Editor Style", nullptr, &gui::windows::showNodeEditorStyleEditor);
    ImGui::MenuItem("ImPlot Style", nullptr, &gui::windows::showImPlotStyleEditor);
    ImGui::MenuItem("Font Size", nullptr, &gui::windows::showFontSizeEditor);
    ImGui::MenuItem("Colormap Editor", nullptr, &gui::windows::showColormapEditor);
}