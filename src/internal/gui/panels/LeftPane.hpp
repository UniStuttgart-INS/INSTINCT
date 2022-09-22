// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LeftPane.hpp
/// @brief Left Pane where Nodes and Selection is shown
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-14

#pragma once

namespace NAV::gui::panels
{
/// @brief Shows the left overview pane
/// @param[in] paneWidth Width of the pane
/// @return True if the pane is active
bool ShowLeftPane(float paneWidth);

} // namespace NAV::gui::panels
