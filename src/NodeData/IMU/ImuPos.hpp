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
    /// @brief Default constructor
    ImuPos() = default;
    /// @brief Destructor
    ~ImuPos() = default;
    /// @brief Copy constructor
    ImuPos(const ImuPos&) = delete;
    /// @brief Move constructor
    ImuPos(ImuPos&&) = delete;
    /// @brief Copy assignment operator
    ImuPos& operator=(const ImuPos&) = delete;
    /// @brief Move assignment operator
    ImuPos& operator=(ImuPos&&) = delete;

    /// Accelerometer position in body frame coordinates in [m]
    const Eigen::Vector3d& posAccel_b() const
    {
        return positionAccel_b;
    }
    /// Gyroscope position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& posGyro_b() const
    {
        return positionGyro_b;
    }
    /// Magnetometer position in body frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& posMag_b() const
    {
        return positionMag_b;
    }

    /// Quaternion from accelerometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatAccel_bp() const
    {
        return quaternionAccel_bp;
    }
    /// Quaternion from body frame to accelerometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatAccel_pb() const
    {
        return quaternionAccel_bp.conjugate();
    }

    /// Quaternion from gyroscope platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatGyro_bp() const
    {
        return quaternionGyro_bp;
    }
    /// Quaternion from body frame to gyroscope platform frame
    [[nodiscard]] Eigen::Quaterniond quatGyro_pb() const
    {
        return quaternionGyro_bp.conjugate();
    }

    /// Quaternion from magnetometer platform frame to body frame
    [[nodiscard]] const Eigen::Quaterniond& quatMag_bp() const
    {
        return quaternionMag_bp;
    }
    /// Quaternion from body frame to magnetometer platform frame
    [[nodiscard]] Eigen::Quaterniond quatMag_pb() const
    {
        return quaternionMag_bp.conjugate();
    }

  private:
    /// Accelerometer position in body frame coordinates in [m]
    Eigen::Vector3d positionAccel_b = { 0, 0, 0 };
    /// Gyroscope position in body frame coordinates in [m]
    Eigen::Vector3d positionGyro_b = { 0, 0, 0 };
    /// Magnetometer position in body frame coordinates in [m]
    Eigen::Vector3d positionMag_b = { 0, 0, 0 };

    /// Quaternion from accelerometer platform frame to body frame
    Eigen::Quaterniond quaternionAccel_bp = { 1, 0, 0, 0 };
    /// Quaternion from gyroscope platform frame to body frame
    Eigen::Quaterniond quaternionGyro_bp = { 1, 0, 0, 0 };
    /// Quaternion from magnetometer platform frame to body frame
    Eigen::Quaterniond quaternionMag_bp = { 1, 0, 0, 0 };

    friend class Imu;
    friend void from_json(const json& j, ImuPos& pos);
};

void to_json(json& j, const ImuPos& pos);
void from_json(const json& j, ImuPos& pos);

} // namespace NAV
