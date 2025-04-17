// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuPos.hpp
/// @brief Imu Position Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-11

#pragma once

#include "util/Eigen.hpp"

namespace NAV
{
// Forward declarations
class Imu;
class MultiImuFile;

/// IMU Position
class ImuPos
{
  public:
    /// IMU position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& b_positionIMU_p() const
    {
        return _b_positionIMU_p;
    }

    /// Quaternion from IMU platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& b_quat_p() const
    {
        return _b_quat_p;
    }
    /// Quaternion from body frame to IMU platform frame
    [[nodiscard]] Eigen::Quaterniond p_quat_b() const
    {
        return _b_quat_p.conjugate();
    }

  private:
    /// IMU position in body frame coordinates in [m]
    Eigen::Vector3d _b_positionIMU_p = Eigen::Vector3d::Zero();

    /// Quaternion from IMU platform frame to body frame
    Eigen::Quaterniond _b_quat_p = Eigen::Quaterniond::Identity();

    friend class Imu;
    friend class MultiImuFile;
    friend void from_json(const json& j, ImuPos& pos);
};

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] pos Object to read info from
void to_json(json& j, const ImuPos& pos);
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] pos Output object
void from_json(const json& j, ImuPos& pos);

} // namespace NAV
