// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuObsSimulated.hpp
/// @brief Data storage class for simulated IMU observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-11-20

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class ImuObsSimulated final : public ImuObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObsSimulated(const ImuPos& imuPos)
        : ImuObs(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ImuObsSimulated";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// The IMU magnetic field measured in units of [Gauss], given in the NED frame.
    Eigen::Vector3d n_magUncomp;
    /// The IMU acceleration measured in units of [m/s^2], given in the NED frame.
    Eigen::Vector3d n_accelUncomp;
    /// The IMU angular rate measured in units of [rad/s], given in the NED frame.
    Eigen::Vector3d n_gyroUncomp;

    /// The IMU magnetic field measured in units of [Gauss], given in the ECEF frame.
    Eigen::Vector3d e_magUncomp;
    /// The IMU acceleration measured in units of [m/s^2], given in the ECEF frame.
    Eigen::Vector3d e_accelUncomp;
    /// The IMU angular rate measured in units of [rad/s], given in the ECEF frame.
    Eigen::Vector3d e_gyroUncomp;
};

} // namespace NAV
