// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FontSizeEditor.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include <imgui.h>

void NAV::gui::windows::ShowFontSizeEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("Font Size Editor", show))
    {
        ImGui::End();
        return;
    }

    bool useBigDefaultFont = gui::NodeEditorApplication::isUsingBigDefaultFont();
    if (ImGui::Checkbox("Use big default font", &useBigDefaultFont))
    {
        gui::NodeEditorApplication::swapDefaultFont(useBigDefaultFont);
    }

    bool useBigWindowFont = gui::NodeEditorApplication::isUsingBigWindowFont();
    if (ImGui::Checkbox("Use big window font", &useBigWindowFont))
    {
        gui::NodeEditorApplication::swapWindowFont(useBigWindowFont);
    }

    bool useBigMonoFont = gui::NodeEditorApplication::isUsingBigMonoFont();
    if (ImGui::Checkbox("Use big mono font", &useBigMonoFont))
    {
        gui::NodeEditorApplication::swapMonoFont(useBigMonoFont);
    }

    // bool useBigHeaderFont = gui::NodeEditorApplication::isUsingBigHeaderFont();
    // if (ImGui::Checkbox("Use big header font", &useBigHeaderFont))
    // {
    //     gui::NodeEditorApplication::swapHeaderFont(useBigHeaderFont);
    // }

    ImGui::End();
}