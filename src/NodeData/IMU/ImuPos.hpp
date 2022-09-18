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
// Forward declaration
class Imu;

/// IMU Position
class ImuPos
{
  public:
    /// Accelerometer position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& b_positionAccel() const
    {
        return _b_positionAccel;
    }
    /// Gyroscope position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& b_positionGyro() const
    {
        return _b_positionGyro;
    }
    /// Magnetometer position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& b_positionMag() const
    {
        return _b_positionMag;
    }

    /// Quaternion from accelerometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& b_quatAccel_p() const
    {
        return _b_quatAccel_p;
    }
    /// Quaternion from body frame to accelerometer platform frame
    [[nodiscard]] Eigen::Quaterniond p_quatAccel_b() const
    {
        return _b_quatAccel_p.conjugate();
    }

    /// Quaternion from gyroscope platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& b_quatGyro_p() const
    {
        return _b_quatGyro_p;
    }
    /// Quaternion from body frame to gyroscope platform frame
    [[nodiscard]] Eigen::Quaterniond p_quatGyro_b() const
    {
        return _b_quatGyro_p.conjugate();
    }

    /// Quaternion from magnetometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& b_quatMag_p() const
    {
        return _b_quatMag_p;
    }
    /// Quaternion from body frame to magnetometer platform frame
    [[nodiscard]] Eigen::Quaterniond p_quatMag_b() const
    {
        return _b_quatMag_p.conjugate();
    }

  private:
    /// Accelerometer position in body frame coordinates in [m]
    Eigen::Vector3d _b_positionAccel = { 0, 0, 0 };
    /// Gyroscope position in body frame coordinates in [m]
    Eigen::Vector3d _b_positionGyro = { 0, 0, 0 };
    /// Magnetometer position in body frame coordinates in [m]
    Eigen::Vector3d _b_positionMag = { 0, 0, 0 };

    /// Quaternion from accelerometer platform frame to body frame
    Eigen::Quaterniond _b_quatAccel_p = Eigen::Quaterniond::Identity();
    /// Quaternion from gyroscope platform frame to body frame
    Eigen::Quaterniond _b_quatGyro_p = { 1, 0, 0, 0 };
    /// Quaternion from magnetometer platform frame to body frame
    Eigen::Quaterniond _b_quatMag_p = { 1, 0, 0, 0 };

    friend class Imu;
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
