// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DebugMenu.hpp"

#include <imgui.h>

#include "internal/gui/windows/Global.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/gui/NodeEditorApplication.hpp"

void NAV::gui::menus::ShowDebugMenu()
{
    ImGui::MenuItem("Show ImGui Demo Window", nullptr, &gui::windows::showImGuiDemoWindow);
    ImGui::MenuItem("Show ImPlot Demo Window", nullptr, &gui::windows::showImPlotDemoWindow);

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    // TODO: The flow animations currently crash under windows
    ImGui::BeginDisabled();
#endif

    ImGui::Checkbox("Show Callback Flow", &nm::showFlowWhenInvokingCallbacks);

    ImGui::Checkbox("Show Notify Flow", &nm::showFlowWhenNotifyingValueChange);

    ImGui::Checkbox("Show Queue size on pins", &NodeEditorApplication::_showQueueSizeOnPins);

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    ImGui::EndDisabled();
#endif
}