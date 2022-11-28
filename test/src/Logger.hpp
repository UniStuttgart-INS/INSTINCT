// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Logger.hpp
/// @brief Logging utilities for testing
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-11-28

#pragma once

#include "util/Logger.hpp"

namespace NAV::TESTS
{

/// @brief Initializes the logger for testing
[[nodiscard]] Logger initializeTestLogger();

} // namespace NAV::TESTS