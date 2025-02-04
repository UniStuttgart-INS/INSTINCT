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

#include <cstdint>
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
        Eigen::Vector3d e_pos; ///< The WGS84 ECEF position of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
    };
    /// Satellite Position and Velocity
    struct PosVel
    {
        Eigen::Vector3d e_pos; ///< The WGS84 ECEF position of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
        Eigen::Vector3d e_vel; ///< The WGS84 ECEF velocity of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
    };
    /// Satellite Position, Velocity and Acceleration
    struct PosVelAccel
    {
        Eigen::Vector3d e_pos;   ///< The WGS84 ECEF position of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
        Eigen::Vector3d e_vel;   ///< The WGS84 ECEF velocity of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
        Eigen::Vector3d e_accel; ///< The WGS84 ECEF acceleration of the satellite at transmit time of the signal, in ECEF axes at the time of reception [m]
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

    /// @brief Calculates position of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    [[nodiscard]] Pos calcSatellitePos(const InsTime& transTime) const;
    /// @brief Calculates position and velocity of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position and velocity for
    [[nodiscard]] PosVel calcSatellitePosVel(const InsTime& transTime) const;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position, velocity and acceleration for
    [[nodiscard]] PosVelAccel calcSatellitePosVelAccel(const InsTime& transTime) const;

    /// @brief Calculates the Variance of the satellite position in [m^2]
    [[nodiscard]] virtual double calcSatellitePositionVariance() const = 0;

  protected:
    /// @brief Calculation flags
    enum Calc : uint8_t
    {
        Calc_None = 0b000,         ///< None
        Calc_Position = 0b001,     ///< Position calculation flag
        Calc_Velocity = 0b010,     ///< Velocity calculation flag
        Calc_Acceleration = 0b100, ///< Acceleration calculation flag
    };

  public:
    friend Orbit::Calc operator|(Orbit::Calc lhs, Orbit::Calc rhs);
    friend Orbit::Calc operator&(Orbit::Calc lhs, Orbit::Calc rhs);

  protected:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite data for
    /// @param[in] calc Flags which determine what should be calculated and returned
    [[nodiscard]] virtual PosVelAccel calcSatelliteData(const InsTime& transTime, Calc calc) const = 0;
};

/// @brief Allows construction of Calc objects
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
inline Orbit::Calc operator|(Orbit::Calc lhs, Orbit::Calc rhs) { return static_cast<Orbit::Calc>(static_cast<int>(lhs) | static_cast<int>(rhs)); }

/// @brief Allows construction of Calc objects
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
inline Orbit::Calc operator&(Orbit::Calc lhs, Orbit::Calc rhs) { return static_cast<Orbit::Calc>(static_cast<int>(lhs) & static_cast<int>(rhs)); }

} // namespace NAV
