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
    [[nodiscard]] const Eigen::Vector3d& posAccel_b() const
    {
        return _positionAccel_b;
    }
    /// Gyroscope position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& posGyro_b() const
    {
        return _positionGyro_b;
    }
    /// Magnetometer position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& posMag_b() const
    {
        return _positionMag_b;
    }

    /// Quaternion from accelerometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatAccel_bp() const
    {
        return _quaternionAccel_bp;
    }
    /// Quaternion from body frame to accelerometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatAccel_pb() const
    {
        return _quaternionAccel_bp.conjugate();
    }

    /// Quaternion from gyroscope platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatGyro_bp() const
    {
        return _quaternionGyro_bp;
    }
    /// Quaternion from body frame to gyroscope platform frame
    [[nodiscard]] Eigen::Quaterniond quatGyro_pb() const
    {
        return _quaternionGyro_bp.conjugate();
    }

    /// Quaternion from magnetometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatMag_bp() const
    {
        return _quaternionMag_bp;
    }
    /// Quaternion from body frame to magnetometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatMag_pb() const
    {
        return _quaternionMag_bp.conjugate();
    }

  private:
    /// Accelerometer position in body frame coordinates in [m]
    Eigen::Vector3d _positionAccel_b = { 0, 0, 0 };
    /// Gyroscope position in body frame coordinates in [m]
    Eigen::Vector3d _positionGyro_b = { 0, 0, 0 };
    /// Magnetometer position in body frame coordinates in [m]
    Eigen::Vector3d _positionMag_b = { 0, 0, 0 };

    /// Quaternion from accelerometer platform frame to body frame
    Eigen::Quaterniond _quaternionAccel_bp = Eigen::Quaterniond::Identity();
    /// Quaternion from gyroscope platform frame to body frame
    Eigen::Quaterniond _quaternionGyro_bp = { 1, 0, 0, 0 };
    /// Quaternion from magnetometer platform frame to body frame
    Eigen::Quaterniond _quaternionMag_bp = { 1, 0, 0, 0 };

    friend class Imu;
    friend void from_json(const json& j, ImuPos& pos);
};

void to_json(json& j, const ImuPos& pos);
void from_json(const json& j, ImuPos& pos);

} // namespace NAV
