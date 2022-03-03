#include "CoordinateFrames.hpp"

#include "util/Logger.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"

namespace NAV::trafo
{
// ###########################################################################################################
//                                             Private functions
// ###########################################################################################################

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates
/// @param[in] lla_position [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return The ECEF coordinates in [m]
/// @note See C. Jekeli, 2001, Inertial Navigation Systems with Geodetic Applications. pp. 23
[[nodiscard]] Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla_position, double a, double e_squared);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude
/// @param[in] e_position Vector with coordinates in ECEF frame in [m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] b Semi-minor axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
/// @note See See S. Gleason (2009) - GNSS Applications and Methods: Software example 'Chapter6_GNSS_INS_1/wgsxyz2lla.m' (J.A. Farrel and M. Barth, 1999, GPS & Inertal Navigation. McGraw-Hill. pp. 29.)
[[nodiscard]] Eigen::Vector3d ecef2lla(const Eigen::Vector3d& e_position, double a, double b, double e_squared); // TODO: Take "Exact conversion of earth-centered, earth-fixed coordinates to geodetic coordinates" by Jijie Zhu instead of Gleason's Matlab code

// ###########################################################################################################
//                                             Public functions
// ###########################################################################################################

Eigen::Vector3d quat2eulerZYX(const Eigen::Quaterniond& q)
{
    // Given range [-pi:pi] x [-pi:pi] x [0:pi]
    Eigen::Vector3d XYZ = q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    // Wanted range (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    if (XYZ.y() >= M_PI / 2.0 || XYZ.y() <= -M_PI / 2.0)
    {
        double x = XYZ.x() > 0 ? XYZ.x() - M_PI : XYZ.x() + M_PI;
        double y = XYZ.y() >= M_PI / 2.0 ? -(XYZ.y() - M_PI) : -(XYZ.y() + M_PI);
        double z = XYZ.z() - M_PI;

        XYZ = { x, y, z };
    }

    return XYZ;
}

Eigen::Quaterniond e_Quat_i(const double time, const double omega_ie)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd zAngle(-omega_ie * time, Eigen::Vector3d::UnitZ());

    return Eigen::Quaterniond(zAngle).normalized();
}

Eigen::Quaterniond i_Quat_e(const double time, const double omega_ie)
{
    return e_Quat_i(time, omega_ie).conjugate();
}

Eigen::Quaterniond e_Quat_n(const double latitude, const double longitude)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxisd longitudeAngle(longitude, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd latitudeAngle(-M_PI_2 - latitude, Eigen::Vector3d::UnitY());

    return (longitudeAngle * latitudeAngle).normalized();
}

Eigen::Quaterniond n_Quat_e(const double latitude, const double longitude)
{
    return e_Quat_n(latitude, longitude).conjugate();
}

Eigen::Quaterniond b_Quat_n(const double roll, const double pitch, const double yaw)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxisd rollAngle(-roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());

    return (rollAngle * pitchAngle * yawAngle).normalized();
}

Eigen::Quaterniond b_Quat_n(const Eigen::Vector3d& rollPitchYaw)
{
    return b_Quat_n(rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));
}

Eigen::Quaterniond n_Quat_b(const double roll, const double pitch, const double yaw)
{
    return b_Quat_n(roll, pitch, yaw).conjugate();
}

Eigen::Quaterniond n_Quat_b(const Eigen::Vector3d& rollPitchYaw)
{
    return n_Quat_b(rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));
}

Eigen::Quaterniond b_Quat_p(double mountingAngleX, double mountingAngleY, double mountingAngleZ)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd xAngle(-mountingAngleX, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yAngle(-mountingAngleY, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd zAngle(-mountingAngleZ, Eigen::Vector3d::UnitZ());

    return (xAngle * yAngle * zAngle).normalized();
}

Eigen::Quaterniond p_Quat_b(double mountingAngleX, double mountingAngleY, double mountingAngleZ)
{
    return b_Quat_p(mountingAngleX, mountingAngleY, mountingAngleZ).conjugate();
}

Eigen::Vector3d ecef2ned(const Eigen::Vector3d& e_position, const Eigen::Vector3d& lla_position_ref)
{
    const auto& latitude_ref = lla_position_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = lla_position_ref(1); // λ Geodetic longitude

    auto e_position_ref = lla2ecef_WGS84(lla_position_ref);

    Eigen::Matrix3d R_ne;
    // clang-format off
    R_ne << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(latitude_ref),
                     -std::sin(longitude_ref)                ,              std::cos(longitude_ref)             ,            0           ,
            -std::cos(latitude_ref) * std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref), -std::sin(latitude_ref);
    // clang-format on

    return R_ne * (e_position - e_position_ref);
}

Eigen::Vector3d ned2ecef(const Eigen::Vector3d& n_position, const Eigen::Vector3d& lla_position_ref)
{
    const auto& latitude_ref = lla_position_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = lla_position_ref(1); // λ Geodetic longitude

    auto e_position_ref = lla2ecef_WGS84(lla_position_ref);

    Eigen::Matrix3d R_en;
    // clang-format off
    R_en << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(longitude_ref), -std::cos(latitude_ref) * std::cos(longitude_ref),
            -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref),
                         std::cos(latitude_ref)              ,             0           ,                -std::sin(latitude_ref)           ;
    // clang-format on

    return e_position_ref + R_en * n_position;
}

Eigen::Vector3d lla2ecef(const Eigen::Vector3d& lla_position, double a, double e_squared)
{
    const auto& latitude = lla_position(0);  // 𝜙 Geodetic latitude
    const auto& longitude = lla_position(1); // λ Geodetic longitude
    const auto& altitude = lla_position(2);  // Altitude (Height above ground)

    // Radius of curvature of the ellipsoid in the prime vertical plane,
    // i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
    double R_E = calcEarthRadius_E(latitude, a, e_squared);

    // Jekeli, 2001 (eq. 1.80) (see  Torge, 1991, for further details)
    return { (R_E + altitude) * std::cos(latitude) * std::cos(longitude),
             (R_E + altitude) * std::cos(latitude) * std::sin(longitude),
             (R_E * (1 - e_squared) + altitude) * std::sin(latitude) };
}

Eigen::Vector3d lla2ecef_WGS84(const Eigen::Vector3d& lla_position)
{
    return lla2ecef(lla_position, InsConst::WGS84_a, InsConst::WGS84_e_squared);
}

Eigen::Vector3d lla2ecef_GRS80(const Eigen::Vector3d& lla_position)
{
    return lla2ecef(lla_position, InsConst::GRS80_a, InsConst::GRS80_e_squared);
}

Eigen::Vector3d ecef2lla(const Eigen::Vector3d& e_position, double a, double b, double e_squared)
{
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

Eigen::Vector3d ecef2lla_WGS84(const Eigen::Vector3d& e_position)
{
    return ecef2lla(e_position, InsConst::WGS84_a, InsConst::WGS84_b, InsConst::WGS84_e_squared);
}

Eigen::Vector3d ecef2lla_GRS80(const Eigen::Vector3d& e_position)
{
    return ecef2lla(e_position, InsConst::GRS80_a, InsConst::GRS80_b, InsConst::GRS80_e_squared);
}

Eigen::Vector3d sph2ecef(const Eigen::Vector3d& position_s, const double& elevation, const double& azimuth)
{
    Eigen::Matrix3d R_se;
    R_se << std::sin(elevation) * std::cos(azimuth), std::cos(elevation) * std::cos(azimuth), -std::sin(azimuth),
        std::sin(elevation) * std::sin(azimuth), std::cos(elevation) * std::sin(azimuth), std::cos(azimuth),
        std::cos(elevation), -std::sin(elevation), 0.0;

    return R_se * position_s;
}

} // namespace NAV::trafo