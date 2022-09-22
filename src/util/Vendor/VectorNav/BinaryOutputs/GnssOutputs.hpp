// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeOutputs.hpp
/// @brief Binary Group 4 – GNSS1 Outputs / Binary Group 7 – GNSS2 Outputs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#include <cstdint>
#include <util/Eigen.hpp>

#include <vn/types.h>
#include "util/Vendor/VectorNav/VectorNavTypes.hpp"

namespace NAV::vendor::vectornav
{
/// @brief Binary Group 4 – GNSS1 Outputs / Binary Group 7 – GNSS2 Outputs
struct GnssOutputs
{
    /// @brief Available data in this struct
    vn::protocol::uart::GpsGroup gnssField = vn::protocol::uart::GpsGroup::GPSGROUP_NONE;

    /// @brief GPS UTC Time
    ///
    /// The current UTC time. The year is given as a signed byte year offset from the year 2000. For example the year 2013 would be given as year 13.
    UTC timeUtc{};

    /// @brief GPS time of week
    ///
    /// The GPS time of week given in nano seconds.
    uint64_t tow{};

    /// @brief GPS week
    ///
    /// The current GPS week.
    uint16_t week{};

    /// @brief Number of tracked satellites
    ///
    /// The number of tracked GNSS satellites.
    uint8_t numSats{};

    /// @brief GNSS fix
    ///
    /// The current GNSS fix.
    /// 0 - No fix
    /// 1 - Time only
    /// 2 - 2D
    /// 3 - 3D
    /// 4 - SBAS
    /// 7 - RTK Float (only GNSS1)
    /// 8 - RTK Fixed (only GNSS1)
    uint8_t fix{};

    /// @brief GNSS position (latitude, longitude, altitude)
    ///
    /// The current GNSS position measurement given as the geodetic latitude, longitude and altitude above the
    /// ellipsoid. The units are in [deg, deg, m] respectively.
    Eigen::Vector3d posLla{};

    /// @brief GNSS position (ECEF)
    ///
    /// The current GNSS position given in the Earth centered Earth fixed (ECEF) coordinate frame, given in meters.
    Eigen::Vector3d posEcef{};

    /// @brief GNSS velocity (NED)
    ///
    /// The current GNSS velocity in the North East Down (NED) coordinate frame, given in m/s.
    Eigen::Vector3f velNed{};

    /// @brief GNSS velocity (ECEF)
    ///
    /// The current GNSS velocity in the Earth centered Earth fixed (ECEF) coordinate frame, given in m/s.
    Eigen::Vector3f velEcef{};

    /// @brief GNSS position uncertainty (NED)
    ///
    /// The current GNSS position uncertainty in the North East Down (NED) coordinate frame, given in meters (1 Sigma).
    Eigen::Vector3f posU{};

    /// @brief GNSS velocity uncertainty
    ///
    /// The current GNSS velocity uncertainty, given in m/s (1 Sigma).
    float velU{};

    /// @brief GNSS time uncertainty
    ///
    /// The current GPS time uncertainty, given in seconds (1 Sigma).
    float timeU{};

    /// @brief GNSS time status and leap seconds
    ///
    /// Flags for valid GPS TOW, week number and UTC and current leap seconds.
    TimeInfo timeInfo{};

    /// @brief Dilution of precision values
    DOP dop{};

    /// @brief Satellite Information
    ///
    /// Information and measurements pertaining to each GNSS satellite in view.
    SatInfo satInfo{};

    /// @brief GNSS Raw Measurements
    ///
    /// Raw measurements pertaining to each GNSS satellite in view.
    RawMeas raw{};
};

} // namespace NAV::vendor::vectornav