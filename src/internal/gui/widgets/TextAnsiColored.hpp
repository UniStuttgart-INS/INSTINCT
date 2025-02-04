// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TextAnsiColored.hpp
/// @brief Text which can be colored by Ansi codes
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-24
/// @note https://github.com/ocornut/imgui/issues/902#issuecomment-673271288
/// @note https://gist.github.com/ddovod/be210315f285becc6b0e455b775286e1

#pragma once

#include <vector>
#include <string>
#include <imgui.h>
#include <imgui_internal.h>

namespace ImGui
{

/// @brief Displays an unformatted ansi text
/// @param[in] text C-style string pointer
/// @param[in] text_end Pointer to the end of the text or nullptr
void TextAnsiUnformatted(const char* text, const char* text_end = nullptr);

/// @brief Displays an ansi text with format string
/// @param[in] fmt Format string
/// @param[in] args Format arguments
void TextAnsiV(const char* fmt, va_list args);

/// @brief Displays an ansi text with format string
/// @param[in] col Color to display text in
/// @param[in] fmt Format string
/// @param[in] args Format arguments
void TextAnsiColoredV(const ImVec4& col, const char* fmt, va_list args);

/// @brief Displays an ansi text with format string
/// @param[in] col Color to display text in
/// @param[in] fmt Format string
/// @param[in] ... Further arguments
void TextAnsiColored(const ImVec4& col, const char* fmt, ...);

} // namespace ImGui