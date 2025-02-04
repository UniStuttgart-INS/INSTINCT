// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file HelpMarker.hpp
/// @brief Text Help Marker (?) with Tooltip
/// @author T. Topp (topp@ins.uni-stuttgart.de)
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
/// @param[in] textWrapLength Length to wrap symbols after (set to 0.0F to disable)
bool BeginHelpMarker(const char* symbol = "(?)", float textWrapLength = 35.0F);

/// @brief Ends a Text Help Marker with custom content
/// @param[in] wrapText Wether text was wrapped
void EndHelpMarker(bool wrapText = true);

} // namespace NAV::gui::widgets
