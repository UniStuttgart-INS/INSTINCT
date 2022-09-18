// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Sleep.hpp
/// @brief Class to catch system signals and sleep
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-10

#pragma once

#include <cstddef>

namespace NAV::Sleep
{
/// @brief Wait the thread till sigusr signal is send
void waitForSignal(bool showText = false);

/// @brief Wait the thread till time passes
/// @param[in] seconds Time to sleep in seconds
void countDownSeconds(size_t seconds);

} // namespace NAV::Sleep
