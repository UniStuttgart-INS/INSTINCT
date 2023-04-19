// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ReceiverClock.hpp
/// @brief Receiver Clock information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-02-07

#pragma once

#include <unordered_map>

#include "util/Container/UncertainValue.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"

namespace NAV
{

/// Receiver Clock information
struct ReceiverClock
{
    /// Estimated receiver clock bias [s]
    UncertainValue<double> bias{ 0.0, 0.0 };
    /// Estimated receiver clock drift [s/s]
    UncertainValue<double> drift{ 0.0, 0.0 };

    /// System time reference system
    SatelliteSystem referenceTimeSatelliteSystem = SatSys_None;
    /// System time differences [s]
    std::unordered_map<SatelliteSystem, UncertainValue<double>> sysTimeDiff;
    /// System time drift difference [s/s]
    std::unordered_map<SatelliteSystem, UncertainValue<double>> sysDriftDiff;
};

} // namespace NAV