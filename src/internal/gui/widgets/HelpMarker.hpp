/// @file HelpMarker.hpp
/// @brief Text Help Marker (?) with Tooltip
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-13

#pragma once

namespace NAV::gui::widgets
{
/// @brief Text Help Marker, e.g. '(?)', with Tooltip
/// @param[in] desc Text to display as tooltip
/// @param[in] symbol Symbol to display
void HelpMarker(const char* desc, const char* symbol = "(?)");

/// @brief Begins a Text Help Marker, e.g. '(?)', with custom content
/// @param[in] symbol Symbol to display
bool BeginHelpMarker(const char* symbol = "(?)");

/// @brief Ends a Text Help Marker with custom content
void EndHelpMarker();

} // namespace NAV::gui::widgets
