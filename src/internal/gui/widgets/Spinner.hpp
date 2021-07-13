/// @file Spinner.hpp
/// @brief Spinner to show that something is done
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-19

#pragma once

#include <imgui.h>

namespace NAV::gui::widgets
{
void Spinner(const char* label, const ImU32& color, float radius, float thickness = 1.0F);

} // namespace NAV::gui::widgets
