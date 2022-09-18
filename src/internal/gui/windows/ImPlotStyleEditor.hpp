// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImPlotStyleEditor.hpp
/// @brief ImPlot style editor window
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-22

#pragma once

namespace NAV::gui::windows
{
/// @brief Shows a window for editing the style of the ImPlot windows
/// @param[in, out] show Flag which indicates whether the window is shown
void ShowImPlotStyleEditor(bool* show = nullptr);

/// @brief If true, the ImPlot config will be saved into the flow file
extern bool saveConfigInFlow;

/// @brief If true, the ImPlot config from the flow file will be preferred over the global settings file
extern bool prefereFlowOverGlobal;

} // namespace NAV::gui::windows
