// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtkSolution.hpp
/// @brief RTK Node/Algorithm output
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-28

#pragma once

#include <vector>
#include <algorithm>
#include "util/Assert.h"
#include "NodeData/State/PosVel.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"

namespace NAV
{
/// SPP Algorithm output
class RtkSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "RtkSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    // --------------------------------------------------------- Public Members ------------------------------------------------------------

    /// Possible types of the RTK solution
    enum class SolutionType
    {
        None, ///< No solution type specified
        SPP,  ///< Solution calculated via SPP algorithm because of missing data for RTK
        // Int
        // Float
    };

    /// Type of th solution
    SolutionType solType = SolutionType::None;

    /// Amount of satellites used for the position calculation
    size_t nSatellitesPosition = 0;
    /// Amount of satellites used for the velocity calculation
    size_t nSatellitesVelocity = 0;

    /// Estimated receiver clock parameter
    ReceiverClock recvClk = { .bias = { std::nan(""), std::nan("") },
                              .drift = { std::nan(""), std::nan("") },
                              .sysTimeDiff = {},
                              .sysDriftDiff = {} };

    // ------------------------------------------------------------- Getter ----------------------------------------------------------------

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] const std::optional<Eigen::Matrix3d>& e_positionStdev() const { return _e_positionStdev; }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] const std::optional<Eigen::Matrix3d>& n_positionStdev() const { return _n_positionStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] const std::optional<Eigen::Matrix3d>& e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const std::optional<Eigen::Matrix3d>& n_velocityStdev() const { return _n_velocityStdev; }

    // ------------------------------------------------------------- Setter ----------------------------------------------------------------

    /// @brief Set the Position in ECEF coordinates and its standard deviation
    /// @param[in] e_position New Position in ECEF coordinates [m]
    /// @param[in] e_positionStdev Standard deviation of Position in ECEF coordinates [m]
    void setPositionAndStdDev_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_positionStdev)
    {
        setPosition_e(e_position);
        _e_positionStdev = e_positionStdev;
        _n_positionStdev = (n_Quat_e().toRotationMatrix() * _e_positionStdev.value() * n_Quat_e().conjugate().toRotationMatrix()).cwiseAbs();
    }

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityStdev Standard deviation of Velocity in earth coordinates [m/s]
    void setVelocityAndStdDev_e(const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityStdev)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityStdev;
        _n_velocityStdev = n_Quat_e().toRotationMatrix() * _e_velocityStdev.value() * n_Quat_e().conjugate().toRotationMatrix();
    }

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    std::optional<Eigen::Matrix3d> _e_positionStdev;
    /// Standard deviation of Position in local navigation frame coordinates [m]
    std::optional<Eigen::Matrix3d> _n_positionStdev;
    /// Standard deviation of Velocity in earth coordinates [m/s]
    std::optional<Eigen::Matrix3d> _e_velocityStdev;
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    std::optional<Eigen::Matrix3d> _n_velocityStdev;
};

} // namespace NAV
