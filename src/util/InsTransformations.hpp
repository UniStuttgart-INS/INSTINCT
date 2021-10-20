/// @file InsTransformations.hpp
/// @brief Transformation collection
/// @author T. Topp (topp@ins.uni-stuttgart.de)
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

#include "util/Eigen.hpp"

#include "util/InsConstants.hpp"

namespace NAV
{
// TODO: Reenable concepts as soon as they become supported by Apple Clang
namespace Concepts
{
// // Only allow numerical Types
// template<typename T>
// concept Numerical = std::is_arithmetic<T>::value;
} // namespace Concepts

namespace trafo
{
/// @brief Convert Degree to Radians
/// @param[in] deg Value to convert in [deg]
/// @return The converted value in [rad]
template<class T,
         typename = std::enable_if_t<std::is_arithmetic_v<T>>>
[[nodiscard]] constexpr double deg2rad(T deg)
{
    return static_cast<double>(deg) * M_PI / 180.0;
}

/// @brief Convert Radians to Degree
/// @param[in] rad Value to convert in [rad]
/// @return The converted value in [deg]
template<class T,
         typename = std::enable_if_t<std::is_arithmetic_v<T>>>
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
/// @return [angleX, angleY, angleZ]^T vector in [rad]. The returned angles are in the ranges (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
[[nodiscard]] Eigen::Vector3d quat2eulerZYX(const Eigen::Quaterniond& q);

/// @brief Quaternion for rotations from inertial to Earth-centered-Earth-fixed frame
/// @param[in] time Time (t - t0)
/// @param[in] angularRate_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_ei(double time, double angularRate_ie = InsConst::angularVelocity_ie);

/// @brief Quaternion for rotations from Earth-centered-Earth-fixed to inertial frame
/// @param[in] time Time (t - t0)
/// @param[in] angularRate_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_ie(double time, double angularRate_ie = InsConst::angularVelocity_ie);

/// @brief Quaternion for rotations from navigation to Earth-fixed frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_en(double latitude, double longitude);

/// @brief Quaternion for rotations from Earth-fixed to navigation frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_ne(double latitude, double longitude);

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_nb(double roll, double pitch, double yaw);

/// @brief Quaternion for rotations from navigation to body frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_bn(double roll, double pitch, double yaw);

/// @brief Quaternion for rotations from platform to body frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_bp(double mountingAngleX, double mountingAngleY, double mountingAngleZ);

/// @brief Quaternion for rotations from body to platform frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
[[nodiscard]] Eigen::Quaterniond quat_pb(double mountingAngleX, double mountingAngleY, double mountingAngleZ);

/// @brief Converts ECEF coordinates into local NED coordinates
/// @param[in] position_e ECEF coordinates in [m] to convert
/// @param[in] latLonAlt_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return [x_N, x_E, x_D]^T Local NED coordinates in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
[[nodiscard]] Eigen::Vector3d ecef2ned(const Eigen::Vector3d& position_e, const Eigen::Vector3d& latLonAlt_ref);

/// @brief Converts local NED coordinates into ECEF coordinates
/// @param[in] position_n NED coordinates in [m] to convert
/// @param[in] latLonAlt_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return ECEF position in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
[[nodiscard]] Eigen::Vector3d ned2ecef(const Eigen::Vector3d& position_n, const Eigen::Vector3d& latLonAlt_ref);

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using WGS84
/// @param[in] latLonAlt [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d lla2ecef_WGS84(const Eigen::Vector3d& latLonAlt);

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using GRS90
/// @param[in] latLonAlt [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d lla2ecef_GRS80(const Eigen::Vector3d& latLonAlt);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using WGS84
/// @param[in] ecef Vector with coordinates in ECEF frame
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
[[nodiscard]] Eigen::Vector3d ecef2lla_WGS84(const Eigen::Vector3d& ecef);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using GRS90
/// @param[in] ecef Vector with coordinates in ECEF frame in [m]
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
[[nodiscard]] Eigen::Vector3d ecef2lla_GRS80(const Eigen::Vector3d& ecef);

/// @brief Converts spherical Earth-centered-Earth-fixed coordinates into cartesian coordinates
/// @param[in] position_s Position in spherical coordinates to convert
/// @param[in] elevation Elevation in [rad]
/// @param[in] azimuth Azimuth in [rad]
/// @return The ECEF coordinates in [m]
[[nodiscard]] Eigen::Vector3d sph2ecef(const Eigen::Vector3d& position_s, const double& elevation, const double& azimuth);

} // namespace trafo

} // namespace NAV
