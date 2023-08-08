// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file FlowTester.hpp
/// @brief Automatic flow file loading and executing
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-05-15

#pragma once

namespace NAV::TESTS
{

/// @brief Loads and executes the flow
/// @param[in] path Path to the flow file
/// @param[in] useTestDirectories Whether to set the paths to 'test/..' or to the root folder
/// @return true if the execution was successful
bool testFlow(const char* path, bool useTestDirectories = true);

/// @brief Runs general purpose cleanup checks
void runGeneralFlowCleanupChecks();

} // namespace NAV::TESTS