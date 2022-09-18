// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeOutputs.hpp
/// @brief Binary Group 6 – INS Outputs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-01

#pragma once

#ifdef HAS_VECTORNAV_LIBRARY

    #include <cstdint>
    #include <util/Eigen.hpp>

    #include <vn/types.h>
    #include "util/Vendor/VectorNav/VectorNavTypes.hpp"

namespace NAV::vendor::vectornav
{
/// @brief Binary Group 6 – INS Outputs
struct InsOutputs
{
    /// @brief Available data in this struct
    vn::protocol::uart::InsGroup insField = vn::protocol::uart::InsGroup::INSGROUP_NONE;

    /// @brief Ins Status
    ///
    /// The INS status bitfield. See the INS Solution - LLA Register in the INS subsystem for more information on
    /// the individual bits in this field.
    InsStatus insStatus{};

    /// @brief Ins Position (latitude, longitude, altitude)
    ///
    /// The estimated position given as latitude, longitude, and altitude given in [deg, deg, m] respectively.
    Eigen::Vector3d posLla{};

    /// @brief Ins Position (ECEF)
    ///
    /// The estimated position given in the Earth centered Earth fixed (ECEF) frame, reported in meters.
    Eigen::Vector3d posEcef{};

    /// @brief Ins Velocity (Body)
    ///
    /// The estimated velocity in the body frame, given in m/s.
    Eigen::Vector3f velBody{};

    /// @brief Ins Velocity (NED)
    ///
    /// The estimated velocity in the North East Down (NED) frame, given in m/s.
    Eigen::Vector3f velNed{};

    /// @brief Ins Velocity (ECEF)
    ///
    /// The estimated velocity in the Earth centered Earth fixed (ECEF) frame, given in m/s.
    Eigen::Vector3f velEcef{};

    /// @brief Compensated magnetic (ECEF)
    ///
    /// The compensated magnetic measurement in the Earth centered Earth fixed (ECEF) frame, given in Gauss.
    Eigen::Vector3f magEcef{};

    /// @brief Compensated acceleration (ECEF)
    ///
    /// The estimated acceleration (with gravity) reported in m/s^2, given in the Earth centered Earth fixed (ECEF)
    /// frame. The acceleration measurement has been bias compensated by the onboard INS filter. This
    /// measurement is attitude dependent, since the attitude is used to map the measurement from the body frame
    /// into the inertial (ECEF) frame. If the device is stationary and the INS filter is tracking, the measurement
    /// should be nominally equivalent to the gravity reference vector in the inertial frame (ECEF).
    Eigen::Vector3f accelEcef{};

    /// @brief Compensated linear acceleration (no gravity) (ECEF)
    ///
    /// The estimated linear acceleration (without gravity) reported in m/s^2, and given in the Earth centered Earth
    /// fixed (ECEF) frame. This measurement is attitude dependent as the attitude solution is used to map the
    /// measurement from the body frame into the inertial (ECEF) frame. This acceleration measurement has been
    /// bias compensated by the onboard INS filter, and the gravity component has been removed using the current
    /// gravity reference vector estimate. If the device is stationary and the onboard INS filter is tracking, the
    /// measurement will nominally read 0 in all three axes.
    Eigen::Vector3f linearAccelEcef{};

    /// @brief Ins Position Uncertainty
    ///
    /// The estimated uncertainty (1 Sigma) in the current position estimate, given in meters.
    float posU{};

    /// @brief Ins Velocity Uncertainty
    ///
    /// The estimated uncertainty (1 Sigma) in the current velocity estimate, given in m/s.
    float velU{};
};

} // namespace NAV::vendor::vectornav

#endif