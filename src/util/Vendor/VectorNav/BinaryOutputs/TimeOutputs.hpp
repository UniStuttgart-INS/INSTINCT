// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeOutputs.hpp
/// @brief Binary Group 2 – Time Outputs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cstdint>

#include <vn/types.h>
#include "util/Vendor/VectorNav/VectorNavTypes.hpp"

namespace NAV::vendor::vectornav
{
/// @brief Binary Group 2 – Time Outputs
struct TimeOutputs
{
    /// @brief Available data in this struct
    vn::protocol::uart::TimeGroup timeField = vn::protocol::uart::TimeGroup::TIMEGROUP_NONE;

    /// @brief Time since startup.
    ///
    /// The system time since startup measured in nano seconds. The time since startup is based upon the internal
    /// TXCO oscillator for the MCU. The accuracy of the internal TXCO is +/- 20ppm (-40C to 85C).
    uint64_t timeStartup{};

    /// @brief Absolute GPS time.
    ///
    /// The absolute GPS time since start of GPS epoch 1980 expressed in nano seconds.
    uint64_t timeGps{};

    /// @brief Time since start of GPS week.
    ///
    /// The time since the start of the current GPS time week expressed in nano seconds.
    uint64_t gpsTow{};

    /// @brief GPS week.
    ///
    /// The current GPS week.
    uint16_t gpsWeek{};

    /// @brief Time since last SyncIn trigger.
    ///
    /// The time since the last SyncIn event trigger expressed in nano seconds.
    uint64_t timeSyncIn{};

    /// @brief Time since last GPS PPS trigger.
    ///
    /// The time since the last GPS PPS trigger event expressed in nano seconds.
    uint64_t timePPS{};

    /// @brief UTC time.
    ///
    /// The current UTC time. The year is given as a signed byte year offset from the year 2000. For example the
    /// year 2013 would be given as year 13.
    UTC timeUtc{};

    /// @brief SyncIn trigger count.
    ///
    /// The number of SyncIn trigger events that have occurred.
    uint32_t syncInCnt{};

    /// @brief SyncOut trigger count.
    ///
    /// The number of SyncOut trigger events that have occurred.
    uint32_t syncOutCnt{};

    /// @brief Time valid status flags.
    ///
    /// Time valid status flags.
    ///
    ///     Fields: timeOk | dateOk | utcTimeValid | resv | resv | resv | resv | resv
    /// Bit Offset:   0    |   1    |       2      |   3  |   4  |   5  |   6  |   7
    ///
    /// Name         | Description
    /// ------------ | ----------------------------------
    /// timeOk       | 1 – GpsTow is valid.
    /// dateOk       | 1 – TimeGps and GpsWeek are valid.
    /// utcTimeValid | 1 – UTC time is valid.
    /// resv         | Reserved for future use.
    TimeStatus timeStatus{};
};

} // namespace NAV::vendor::vectornav