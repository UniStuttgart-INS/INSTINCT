// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CoordinateFrames.hpp
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

#include <cmath>
#include "util/Eigen.hpp"
#include "util/Logger.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV::trafo
{
namespace internal
{
/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return The ECEF coordinates in [m]
/// @note See C. Jekeli, 2001, Inertial Navigation Systems with Geodetic Applications. pp. 23
template<typename Derived>
Eigen::Vector3<typename Derived::Scalar> lla2ecef(const Eigen::MatrixBase<Derived>& lla_position, typename Derived::Scalar a, typename Derived::Scalar e_squared)
{
    const auto& latitude = lla_position(0);  // 𝜙 Geodetic latitude
    const auto& longitude = lla_position(1); // λ Geodetic longitude
    const auto& altitude = lla_position(2);  // Altitude (Height above ground)

    // Radius of curvature of the ellipsoid in the prime vertical plane,
    // i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
    auto R_E = calcEarthRadius_E(latitude, a, e_squared);

    // Jekeli, 2001 (eq. 1.80) (see  Torge, 1991, for further details)
    return { (R_E + altitude) * std::cos(latitude) * std::cos(longitude),
             (R_E + altitude) * std::cos(latitude) * std::sin(longitude),
             (R_E * (1 - e_squared) + altitude) * std::sin(latitude) };
}

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude
/// @param[in] e_position Vector with coordinates in ECEF frame in [m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] b Semi-minor axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
/// @note See See S. Gleason (2009) - GNSS Applications and Methods: Software example 'Chapter6_GNSS_INS_1/wgsxyz2lla.m' (J.A. Farrel and M. Barth, 1999, GPS & Inertal Navigation. McGraw-Hill. pp. 29.)
template<typename Derived>
Eigen::Vector3<typename Derived::Scalar> ecef2lla(const Eigen::MatrixBase<Derived>& e_position,
                                                  typename Derived::Scalar a, typename Derived::Scalar b, typename Derived::Scalar e_squared)
{
    if (e_position.isZero()) { return { 0, 0, -a }; }

    auto x = e_position(0);
    auto y = e_position(1);
    auto z = e_position(2);

    // Calculate longitude

    auto lon = std::atan2(y, x);

    // Start computing intermediate variables needed to compute altitude

    auto p = e_position.head(2).norm();
    auto E = std::sqrt(a * a - b * b);
    auto F = 54 * std::pow(b * z, 2);
    auto G = p * p + (1 - e_squared) * z * z - e_squared * E * E;
    auto c = e_squared * e_squared * F * p * p / std::pow(G, 3);
    auto s = std::pow(1 + c + std::sqrt(c * c + 2 * c), 1.0 / 3.0);
    auto P = (F / (3 * G * G)) / std::pow(s + (1.0 / s) + 1, 2);
    auto Q = std::sqrt(1 + 2 * e_squared * e_squared * P);
    auto k_1 = -P * e_squared * p / (1 + Q);
    auto k_2 = 0.5 * a * a * (1 + 1 / Q);
    auto k_3 = -P * (1 - e_squared) * z * z / (Q * (1 + Q));
    auto k_4 = -0.5 * P * p * p;
    auto r_0 = k_1 + std::sqrt(k_2 + k_3 + k_4);
    auto k_5 = (p - e_squared * r_0);
    auto U = std::sqrt(k_5 * k_5 + z * z);
    auto V = std::sqrt(k_5 * k_5 + (1 - e_squared) * z * z);

    auto alt = U * (1 - (b * b / (a * V)));

    // Compute additional values required for computing latitude

    auto z_0 = (b * b * z) / (a * V);
    auto e_p = (a / b) * std::sqrt(e_squared);

    auto lat = std::atan((z + z_0 * (e_p * e_p)) / p);

    return { lat, lon, alt };
}
} // namespace internal

/// @brief Converts the quaternion to Euler rotation angles with rotation sequence ZYX
/// @param[in] q Quaternion to convert
/// @return [angleX, angleY, angleZ]^T vector in [rad]. The returned angles are in the ranges (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> quat2eulerZYX(const Eigen::QuaternionBase<Derived>& q)
{
    // Given range [-pi:pi] x [-pi:pi] x [0:pi]
    Eigen::Vector3<typename Derived::Scalar> XYZ = q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    // Wanted range (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    if (XYZ.y() >= M_PI / 2.0 || XYZ.y() <= -M_PI / 2.0)
    {
        typename Derived::Scalar x = XYZ.x() > 0 ? XYZ.x() - M_PI : XYZ.x() + M_PI;
        typename Derived::Scalar y = XYZ.y() >= M_PI / 2.0 ? -(XYZ.y() - M_PI) : -(XYZ.y() + M_PI);
        typename Derived::Scalar z = XYZ.z() - M_PI;

        XYZ = { x, y, z };
    }

    return XYZ;
}

/// @brief Quaternion for rotations from inertial to Earth-centered-Earth-fixed frame
/// @param[in] time Time (t - t0)
/// @param[in] omega_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> e_Quat_i(Scalar time, Scalar omega_ie = InsConst<Scalar>::omega_ie)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxis<Scalar> zAngle(-omega_ie * time, Eigen::Vector3<Scalar>::UnitZ());

    return Eigen::Quaternion<Scalar>(zAngle).normalized();
}

/// @brief Quaternion for rotations from Earth-centered-Earth-fixed to inertial frame
/// @param[in] time Time (t - t0)
/// @param[in] omega_ie Angular velocity in [rad/s] of earth frame with regard to the inertial frame
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> i_Quat_e(Scalar time, Scalar omega_ie = InsConst<Scalar>::omega_ie)
{
    return e_Quat_i(time, omega_ie).conjugate();
}

/// @brief Quaternion for rotations from navigation to Earth-fixed frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> e_Quat_n(Scalar latitude, Scalar longitude)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxis<Scalar> longitudeAngle(longitude, Eigen::Vector3<Scalar>::UnitZ());
    Eigen::AngleAxis<Scalar> latitudeAngle(-M_PI_2 - latitude, Eigen::Vector3<Scalar>::UnitY());

    return (longitudeAngle * latitudeAngle).normalized();
}

/// @brief Quaternion for rotations from Earth-fixed to navigation frame
/// @param[in] latitude 𝜙 Geodetic latitude in [rad]
/// @param[in] longitude λ Geodetic longitude in [rad]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> n_Quat_e(Scalar latitude, Scalar longitude)
{
    return e_Quat_n(latitude, longitude).conjugate();
}

/// @brief Quaternion for rotations from navigation to body frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> b_Quat_n(Scalar roll, Scalar pitch, Scalar yaw)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxis<Scalar> rollAngle(-roll, Eigen::Vector3<Scalar>::UnitX());
    Eigen::AngleAxis<Scalar> pitchAngle(-pitch, Eigen::Vector3<Scalar>::UnitY());
    Eigen::AngleAxis<Scalar> yawAngle(-yaw, Eigen::Vector3<Scalar>::UnitZ());

    return (rollAngle * pitchAngle * yawAngle).normalized();
}

/// @brief Quaternion for rotations from navigation to body frame
/// @param[in] rollPitchYaw Roll, Pitch, Yaw angle in [rad]
/// @return The rotation Quaternion representation
template<typename Derived>
[[nodiscard]] Eigen::Quaternion<typename Derived::Scalar> b_Quat_n(const Eigen::MatrixBase<Derived>& rollPitchYaw)
{
    return b_Quat_n(rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));
}

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] roll Roll angle in [rad]
/// @param[in] pitch Pitch angle in [rad]
/// @param[in] yaw Yaw angle in [rad]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> n_Quat_b(Scalar roll, Scalar pitch, Scalar yaw)
{
    return b_Quat_n(roll, pitch, yaw).conjugate();
}

/// @brief Quaternion for rotations from body to navigation frame
/// @param[in] rollPitchYaw Roll, Pitch, Yaw angle in [rad]
/// @return The rotation Quaternion representation
template<typename Derived>
[[nodiscard]] Eigen::Quaternion<typename Derived::Scalar> n_Quat_b(const Eigen::MatrixBase<Derived>& rollPitchYaw)
{
    return n_Quat_b(rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));
}

/// @brief Quaternion for rotations from platform to body frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> b_Quat_p(Scalar mountingAngleX, Scalar mountingAngleY, Scalar mountingAngleZ)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxis<Scalar> xAngle(-mountingAngleX, Eigen::Vector3<Scalar>::UnitX());
    Eigen::AngleAxis<Scalar> yAngle(-mountingAngleY, Eigen::Vector3<Scalar>::UnitY());
    Eigen::AngleAxis<Scalar> zAngle(-mountingAngleZ, Eigen::Vector3<Scalar>::UnitZ());

    return (xAngle * yAngle * zAngle).normalized();
}

/// @brief Quaternion for rotations from body to platform frame
/// @param[in] mountingAngleX Mounting angle to x axis in [rad]. First rotation. (-pi:pi]
/// @param[in] mountingAngleY Mounting angle to y axis in [rad]. Second rotation. (-pi/2:pi/2]
/// @param[in] mountingAngleZ Mounting angle to z axis in [rad]. Third rotation. (-pi:pi]
/// @return The rotation Quaternion representation
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Quaternion<Scalar> p_Quat_b(Scalar mountingAngleX, Scalar mountingAngleY, Scalar mountingAngleZ)
{
    return b_Quat_p(mountingAngleX, mountingAngleY, mountingAngleZ).conjugate();
}

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using WGS84
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> lla2ecef_WGS84(const Eigen::MatrixBase<Derived>& lla_position)
{
    return internal::lla2ecef(lla_position, InsConst<typename Derived::Scalar>::WGS84::a, InsConst<typename Derived::Scalar>::WGS84::e_squared);
}

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates using GRS90
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @return The ECEF coordinates in [m]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> lla2ecef_GRS80(const Eigen::MatrixBase<Derived>& lla_position)
{
    return internal::lla2ecef(lla_position, InsConst<typename Derived::Scalar>::GRS80::a, InsConst<typename Derived::Scalar>::GRS80::e_squared);
}

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using WGS84
/// @param[in] e_position Vector with coordinates in ECEF frame
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> ecef2lla_WGS84(const Eigen::MatrixBase<Derived>& e_position)
{
    return internal::ecef2lla(e_position,
                              InsConst<typename Derived::Scalar>::WGS84::a,
                              InsConst<typename Derived::Scalar>::WGS84::b,
                              InsConst<typename Derived::Scalar>::WGS84::e_squared);
}

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude using GRS90
/// @param[in] e_position Vector with coordinates in ECEF frame in [m]
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> ecef2lla_GRS80(const Eigen::MatrixBase<Derived>& e_position)
{
    return internal::ecef2lla(e_position,
                              InsConst<typename Derived::Scalar>::GRS80::a,
                              InsConst<typename Derived::Scalar>::GRS80::b,
                              InsConst<typename Derived::Scalar>::GRS80::e_squared);
}

/// @brief Converts ECEF coordinates into local NED coordinates
/// @param[in] e_position ECEF coordinates in [m] to convert
/// @param[in] lla_position_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return [x_N, x_E, x_D]^T Local NED coordinates in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
template<typename DerivedA, typename DerivedB>
[[nodiscard]] Eigen::Vector3<typename DerivedA::Scalar> ecef2ned(const Eigen::MatrixBase<DerivedA>& e_position, const Eigen::MatrixBase<DerivedB>& lla_position_ref)
{
    const auto& latitude_ref = lla_position_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = lla_position_ref(1); // λ Geodetic longitude

    auto e_position_ref = lla2ecef_WGS84(lla_position_ref);

    Eigen::Matrix3<typename DerivedA::Scalar> R_ne;
    // clang-format off
    R_ne << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(latitude_ref),
                     -std::sin(longitude_ref)                ,              std::cos(longitude_ref)             ,            0           ,
            -std::cos(latitude_ref) * std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref), -std::sin(latitude_ref);
    // clang-format on

    return R_ne * (e_position - e_position_ref);
}

/// @brief Converts local NED coordinates into ECEF coordinates
/// @param[in] n_position NED coordinates in [m] to convert
/// @param[in] lla_position_ref Reference [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m] which represents the origin of the local frame
/// @return ECEF position in [m]
/// @note See G. Cai, B.M. Chen, Lee, T.H. Lee, 2011, Unmanned Rotorcraft Systems. Springer. pp. 32
/// @attention This function does not take the sphericity of the Earth into account
template<typename DerivedA, typename DerivedB>
[[nodiscard]] Eigen::Vector3<typename DerivedA::Scalar> ned2ecef(const Eigen::MatrixBase<DerivedA>& n_position, const Eigen::MatrixBase<DerivedB>& lla_position_ref)
{
    const auto& latitude_ref = lla_position_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = lla_position_ref(1); // λ Geodetic longitude

    auto e_position_ref = lla2ecef_WGS84(lla_position_ref);

    Eigen::Matrix3<typename DerivedA::Scalar> R_en;
    // clang-format off
    R_en << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(longitude_ref), -std::cos(latitude_ref) * std::cos(longitude_ref),
            -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref),
                         std::cos(latitude_ref)              ,             0           ,                -std::sin(latitude_ref)           ;
    // clang-format on

    return e_position_ref + R_en * n_position;
}

/// @brief Converts PZ-90.11 coordinates to WGS84 coordinates
/// @param[in] pz90_pos Position in PZ-90.11 coordinates
/// @return Position in WGS84 coordinates
/// @note See \cite PZ-90.11 PZ-90.11 Reference Document Appendix 4, p.34f
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> pz90toWGS84_pos(const Eigen::MatrixBase<Derived>& pz90_pos)
{
    typename Derived::Scalar m = -0.008e-6;
    auto omega_x = static_cast<typename Derived::Scalar>(-2.3_mas);
    auto omega_y = static_cast<typename Derived::Scalar>(3.54_mas);
    auto omega_z = static_cast<typename Derived::Scalar>(-4.21_mas);
    Eigen::Vector3<typename Derived::Scalar> dX{ -0.013, 0.106, 0.022 };

    Eigen::Matrix3<typename Derived::Scalar> T;
    T << 1, -omega_z, omega_y,
        omega_z, 1, -omega_x,
        -omega_y, omega_x, 1;

    return 1.0 / (1.0 + m) * T * (pz90_pos - dX);
}

/// @brief Converts PZ-90.11 vectors to WGS84 frame
/// @param[in] pz90 Vector in PZ-90.11 frame
/// @param[in] pz90_pos Position in PZ-90.11 frame (needed for calculation)
/// @return Vector in WGS84 frame
template<typename DerivedA, typename DerivedB>
[[nodiscard]] Eigen::Vector3<typename DerivedA::Scalar> pz90toWGS84(const Eigen::MatrixBase<DerivedA>& pz90, const Eigen::MatrixBase<DerivedB>& pz90_pos)
{
    return pz90toWGS84_pos(pz90_pos + pz90) - pz90toWGS84_pos(pz90_pos);
}

/// @brief Converts spherical Earth-centered-Earth-fixed coordinates into cartesian coordinates
/// @param[in] position_s Position in spherical coordinates to convert
/// @param[in] elevation Elevation in [rad]
/// @param[in] azimuth Azimuth in [rad]
/// @return The ECEF coordinates in [m]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> sph2ecef(const Eigen::MatrixBase<Derived>& position_s,
                                                                const typename Derived::Scalar& elevation,
                                                                const typename Derived::Scalar& azimuth)
{
    Eigen::Matrix3<typename Derived::Scalar> R_se;
    R_se << std::sin(elevation) * std::cos(azimuth), std::cos(elevation) * std::cos(azimuth), -std::sin(azimuth),
        std::sin(elevation) * std::sin(azimuth), std::cos(elevation) * std::sin(azimuth), std::cos(azimuth),
        std::cos(elevation), -std::sin(elevation), 0.0;

    return R_se * position_s;
}

} // namespace NAV::trafo
