/// @file FontSizeEditor.hpp
/// @brief Font size chooser window
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-02

#pragma once

namespace NAV::gui::windows
{

/// @brief Shows a window for choosing the font size
/// @param[in, out] show Flag which indicates whether the window is shown
void ShowFontSizeEditor(bool* show = nullptr);

} // namespace NAV::gui::windows
