/// @file HelpMarker.hpp
/// @brief Text Help Marker (?) with Tooltip
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-13

#pragma once

namespace NAV::gui::widgets
{
/// @brief Text Help Marker (?) with Tooltip
/// @param[in] desc Text to display as tooltip
/// @param[in] symbol Symbol to display
void HelpMarker(const char* desc, const char* symbol = "(?)");

} // namespace NAV::gui::widgets
