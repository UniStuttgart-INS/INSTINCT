/// @file Spinner.hpp
/// @brief Spinner to show that something is done
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-19

#pragma once

#include <imgui.h>

namespace NAV::gui::widgets
{
/// @brief Shows a Spinner to signal that work is done
/// @param[in] label Label for the spinner. Is not displayed. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in] color Color of the spinner
/// @param[in] radius Radius of the spinner
/// @param[in] thickness Thickness of the spinner
void Spinner(const char* label, const ImU32& color, float radius, float thickness = 1.0F);

} // namespace NAV::gui::widgets
