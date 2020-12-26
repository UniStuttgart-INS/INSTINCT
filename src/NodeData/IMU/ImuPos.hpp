/// @file ImuPos.hpp
/// @brief Imu Position Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-11

#pragma once

#include "util/LinearAlgebra.hpp"

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
    [[nodiscard]] const Vector3d<Body>& posAccel_b() const
    {
        return positionAccel_b;
    }
    /// Gyroscope position in body frame coordinates in [m]
    [[nodiscard]] const Vector3d<Body>& posGyro_b() const
    {
        return positionGyro_b;
    }
    /// Magnetometer position in body frame coordinates in [m]
    [[nodiscard]] const Vector3d<Body>& posMag_b() const
    {
        return positionMag_b;
    }

    /// Quaternion from accelerometer platform frame to body frame
    [[nodiscard]] const Quaterniond<Body, Platform>& quatAccel_bp() const
    {
        return quaternionAccel_bp;
    }
    /// Quaternion from body frame to accelerometer platform frame
    [[nodiscard]] Quaterniond<Platform, Body> quatAccel_pb() const
    {
        return quaternionAccel_bp.conjugate();
    }

    /// Quaternion from gyroscope platform frame to body frame
    [[nodiscard]] const Quaterniond<Body, Platform>& quatGyro_bp() const
    {
        return quaternionGyro_bp;
    }
    /// Quaternion from body frame to gyroscope platform frame
    [[nodiscard]] Quaterniond<Platform, Body> quatGyro_pb() const
    {
        return quaternionGyro_bp.conjugate();
    }

    /// Quaternion from magnetometer platform frame to body frame
    [[nodiscard]] const Quaterniond<Body, Platform>& quatMag_bp() const
    {
        return quaternionMag_bp;
    }
    /// Quaternion from body frame to magnetometer platform frame
    [[nodiscard]] Quaterniond<Platform, Body> quatMag_pb() const
    {
        return quaternionMag_bp.conjugate();
    }

  private:
    /// Accelerometer position in body frame coordinates in [m]
    Vector3d<Body> positionAccel_b;
    /// Gyroscope position in body frame coordinates in [m]
    Vector3d<Body> positionGyro_b;
    /// Magnetometer position in body frame coordinates in [m]
    Vector3d<Body> positionMag_b;

    /// Quaternion from accelerometer platform frame to body frame
    Quaterniond<Body, Platform> quaternionAccel_bp;
    /// Quaternion from gyroscope platform frame to body frame
    Quaterniond<Body, Platform> quaternionGyro_bp;
    /// Quaternion from magnetometer platform frame to body frame
    Quaterniond<Body, Platform> quaternionMag_bp;

    friend class Imu;
};

} // namespace NAV
