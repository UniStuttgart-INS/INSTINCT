// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AppLogic.hpp
/// @brief Application logic
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-17

#pragma once

namespace NAV::AppLogic
{
/// @brief Processes the command line arguments
/// @param[in] argc Argument Count
/// @param[in] argv Argument Values
int processCommandLineArguments(int argc, const char* argv[]); // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)

} // namespace NAV::AppLogic