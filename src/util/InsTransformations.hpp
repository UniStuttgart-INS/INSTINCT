/// @file InsTransformations.hpp
/// @brief Transformation collection
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-08
///
/// Coordinate Frames:
/// - Inertial frame (i-frame)
///     O_i: Earth center
///     x_i: Direction to Vernal equinox
///     y_i: In equatorial plane, complementing to Right-Hand-System
///     z_i: Vertical on equatorial plane (North)
/// - Earth-centered-Earth-fixed frame (e-frame)
///     O_e: Earth center of mass
///     x_e: Direction to Greenwich meridian (longitude = 0°)
///     y_e: In equatorial plane, complementing to Right-Hand-System
///     z_e: Vertical on equatorial plane (North)
/// - Local Navigation frame (n-frame)
///     O_n: Vehicle center of mass
///     x_n: "North"
///     y_n: Right-Hand-System ("East")
///     z_n: Earth center ("Down")
/// - Body frame (b-frame)
///     O_b: Vehicle center of mass
///     x_b: Roll-axis ("Forward")
///     y_b: Pitch-axis ("Right")
///     z_b: Yaw-axis ("Down")
/// - Platform frame (p-frame)
///     O_b: Center of IMU
///     x_b: X-Axis Accelerometer/Gyrometer
///     y_b: Y-Axis Accelerometer/Gyrometer
///     z_b: Z-Axis Accelerometer/Gyrometer

#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

#include "util/InsConstants.hpp"

namespace NAV::trafo
{
/// Only allow numerical Types
template<typename T>
concept Numerical = std::is_arithmetic<T>::value;

/// @brief Convert Degree to Radians
/// @param[in] deg Value to convert in [deg]
/// @return The converted value in [rad]
template<Numerical T>
[[nodiscard]] constexpr double deg2rad(T deg)
{
    return static_cast<double>(deg) * M_PI / 180.0;
}

/// @brief Convert Radians to Degree
/// @param[in] rad Value to convert in [rad]
/// @return The converted value in [deg]
template<Numerical T>
[[nodiscard]] constexpr double rad2deg(T rad)
{
    return static_cast<double>(rad) * 180.0 / M_PI;
}

/// @brief Convert Degree to Radians
/// @param[in] deg Vector to convert in [deg]
/// @return The converted Vector in [rad]
[[nodiscard]] Eigen::Vector3d deg2rad3(const Eigen::Vector3d& deg);

/// @brief Convert Radians to Degree
/// @param[in] rad Vector to convert in [rad]
/// @return The converted Vector in [deg]
[[nodiscard]] Eigen::Vector3d rad2deg3(const Eigen::Vector3d& rad);

/// @brief Converts the quaternion to Euler rotation angles with rotation sequence ZYX
/// @param[in] q Quaternion to convert
/// @return [angleZ, angleY, angleX]^T vector in [rad]. The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi]
[[nodiscard]] Eigen::Vector3d quat2eulerZYX(const Eigen::Quaterniond& q);

/// @brief Quaternion for rotations from inertial to Earth-centered-Earth-fixed frame
/// @param[in] time Time (t - t0)
/// @param[in] angularRate_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_i2e(double time, double angularRate_ie = InsConst::angularVelocity_ie);

/// @brief Direction cosine matrix for rotations from inertial to Earth-fixed frame
/// @param[in] time Time (t - t0)
/// @param[in] angularRate_ie Angular velocity in [rad/s] of Earth frame with regard to the inertial frame
/// @return The DCM matrix
[[nodiscard]] Eigen::Matrix3d DCM_i2e(double time, double angularRate_ie = InsConst::angularVelocity_ie);

/// @brief Quaternion for rotations from navigation to Earth-fixed frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_n2e(double latitude, double longitude);

/// @brief Direction cosine matrix for rotations from navigation to Earth-fixed frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The DCM matrix
[[nodiscard]] Eigen::Matrix3d DCM_n2e(double latitude, double longitude);

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_b2n(double roll, double pitch, double yaw);

/// @brief Direction cosine matrix for rotations from body to navigation frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @return The DCM matrix
[[nodiscard]] Eigen::Matrix3d DCM_b2n(double roll, double pitch, double yaw);

[[nodiscard]] Eigen::Vector3d llh2ecef_wgs84(double latitude, double longitude, double height);

} // namespace NAV::trafo
