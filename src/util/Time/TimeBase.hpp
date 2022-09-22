// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeBase.hpp
/// @brief Keeps track of the current real/simulation time
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-28

#pragma once

#include "Navigation/Time/InsTime.hpp"

namespace NAV::util::time
{
/// @brief Different Modes the Time Base class can work in
enum class Mode
{
    REAL_TIME,       ///< Computer clock will be added to last time update
    POST_PROCESSING, ///< Time will be set by FlowExecutor only
};

/// @brief Set the time mode
/// @param[in] mode Real time or postprocessing
void SetMode(Mode mode);

/// @brief Get the time mode
Mode GetMode();

/// @brief Get the current time.
/// @return Pointer to the current time or nullptr if it is not known yet.
InsTime GetCurrentInsTime();

/// @brief Set the current time object
/// @param[in] insTime The new current time
void SetCurrentTime(const InsTime& insTime);

/// @brief Clears the current time object
void ClearCurrentTime();

// TODO: Remove this all. Nodes have to give the time to each other over links. Each message needs a time, not globally

} // namespace NAV::util::time
