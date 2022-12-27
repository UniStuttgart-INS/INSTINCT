// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Orbit.hpp
/// @brief Abstract satellite orbit information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-02

#pragma once

#include "Navigation/Time/InsTime.hpp"
#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Abstract satellite orbit information
class Orbit
{
  public:
    /// Satellite Position
    struct Pos
    {
        Eigen::Vector3d e_pos; ///< The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m]
    };
    /// Satellite Position and Velocity
    struct PosVel
    {
        Eigen::Vector3d e_pos; ///< The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m]
        Eigen::Vector3d e_vel; ///< The WGS84 frame velocity of the satellite at the requested time [m/s]
    };
    /// Satellite Position, Velocity and Acceleration
    struct PosVelAccel
    {
        Eigen::Vector3d e_pos;   ///< The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m]
        Eigen::Vector3d e_vel;   ///< The WGS84 frame velocity of the satellite at the requested time [m/s]
        Eigen::Vector3d e_accel; ///< The WGS84 frame acceleration of the satellite at the requested time [m/s^2]
    };

    /// @brief Default Constructor
    Orbit() = default;
    /// @brief Destructor
    virtual ~Orbit() = default;
    /// @brief Copy constructor
    Orbit(const Orbit&) = default;
    /// @brief Move constructor
    Orbit(Orbit&&) = default;
    /// @brief Copy assignment operator
    Orbit& operator=(const Orbit&) = delete;
    /// @brief Move assignment operator
    Orbit& operator=(Orbit&&) = delete;

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] virtual Pos calcSatellitePos(const InsTime& transTime) const = 0;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] virtual PosVel calcSatellitePosVel(const InsTime& transTime) const = 0;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] virtual PosVelAccel calcSatellitePosVelAccel(const InsTime& transTime) const = 0;

    /// @brief Calculates the Variance of the satellite position in [m]
    [[nodiscard]] virtual double calcSatellitePositionVariance() const = 0;
};

} // namespace NAV
