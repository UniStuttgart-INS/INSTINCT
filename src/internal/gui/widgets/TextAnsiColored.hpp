/// @file TextAnsiColored.hpp
/// @brief Text which can be colored by Ansi codes
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-24
/// @note https://github.com/ocornut/imgui/issues/902#issuecomment-673271288
/// @note https://gist.github.com/ddovod/be210315f285becc6b0e455b775286e1

#pragma once

#include <vector>
#include <string>
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>

namespace ImGui
{

void TextAnsiUnformatted(const char* text, const char* text_end = nullptr);

void TextAnsiV(const char* fmt, va_list args);

void TextAnsiColoredV(const ImVec4& col, const char* fmt, va_list args);

void TextAnsiColored(const ImVec4& col, const char* fmt, ...);

} // namespace ImGui