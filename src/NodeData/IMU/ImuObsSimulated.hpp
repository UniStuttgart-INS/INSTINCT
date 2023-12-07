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

    /// The acceleration derived from the trajectory in [m/s^2], given in the NED frame.
    Eigen::Vector3d n_accelDynamics;
    /// The angular rate ω_nb_n derived from the trajectory in [rad/s], given in the NED frame.
    Eigen::Vector3d n_angularRateDynamics;

    /// The acceleration derived from the trajectory in [m/s^2], given in the ECEF frame.
    Eigen::Vector3d e_accelDynamics;
    /// The angular rate ω_nb_e derived from the trajectory in [rad/s], given in the ECEF frame.
    Eigen::Vector3d e_angularRateDynamics;
};

} // namespace NAV
