// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Screenshotter.hpp
/// @brief Screenshot utility
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-31

#pragma once

#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W

    #include <string>

namespace NAV::gui::windows
{

/// @brief Shows a window for taking screenshots
/// @param[in, out] show Flag which indicates whether the window is shown
void ShowScreenshotter(bool* show = nullptr);

/// @brief Copies the file given to the clipboard
/// @param[in] path Path to the file
void CopyFileToClipboard(const char* path);

/// @brief Json file containing the ImPlot style used for taking screenshots
extern std::string plotScreenshotImPlotStyleFile;

/// If true, copy screenshots to clipboard
extern bool copyScreenshotsToClipboard;

} // namespace NAV::gui::windows

#endif