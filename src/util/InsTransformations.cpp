#include "InsTransformations.hpp"

#include "util/Logger.hpp"

namespace NAV::trafo
{
// ###########################################################################################################
//                                             Private functions
// ###########################################################################################################

/// @brief Converts latitude, longitude and altitude into Earth-centered-Earth-fixed coordinates
/// @param[in] latLonAlt [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return The ECEF coordinates in [m]
/// @note See C. Jekeli, 2001, Inertial Navigation Systems with Geodetic Applications. pp. 23
[[nodiscard]] Eigen::Vector3d lla2ecef(const Eigen::Vector3d& latLonAlt, double a, double e_squared);

/// @brief Converts Earth-centered-Earth-fixed coordinates into latitude, longitude and altitude
/// @param[in] ecef Vector with coordinates in ECEF frame in [m]
/// @param[in] a Semi-major axis of the reference ellipsoid
/// @param[in] b Semi-minor axis of the reference ellipsoid
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Vector containing [latitude 𝜙, longitude λ, altitude]^T in [rad, rad, m]
/// @note See See S. Gleason (2009) - GNSS Applications and Methods: Software example 'Chapter6_GNSS_INS_1/wgsxyz2lla.m' (J.A. Farrel and M. Barth, 1999, GPS & Inertal Navigation. McGraw-Hill. pp. 29.)
[[nodiscard]] Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef, double a, double b, double e_squared); //TODO: Take "Exact conversion of earth-centered, earth-fixed coordinates to geodetic coordinates" by Jijie Zhu instead of Gleason's Matlab code

// ###########################################################################################################
//                                             Public functions
// ###########################################################################################################

Eigen::Vector3d deg2rad3(const Eigen::Vector3d& deg)
{
    return deg * M_PI / 180.0;
}

Eigen::Vector3d rad2deg3(const Eigen::Vector3d& rad)
{
    return rad * 180.0 / M_PI;
}

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
#ifndef NDEBUG
        // Wanted range
        if (x > -M_PI && x <= M_PI                // (-pi:pi]
            && y > -M_PI / 2.0 && y <= M_PI / 2.0 // (-pi/2:pi/2]
            && z > -M_PI && z <= M_PI)            // (-pi:pi]
        {
#endif
            XYZ = { x, y, z };
#ifndef NDEBUG
        }
        else
        {
            LOG_ERROR("\nCould not convert the angles [{}, {}, {}]", XYZ.x(), XYZ.y(), XYZ.z());
        }
#endif
    }

    return XYZ;
}

Eigen::Quaterniond quat_ei(const double time, const double angularRate_ie)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd zAngle(-angularRate_ie * time, Eigen::Vector3d::UnitZ());

    return Eigen::Quaterniond(zAngle).normalized();
}

Eigen::Quaterniond quat_ie(const double time, const double angularRate_ie)
{
    return quat_ei(time, angularRate_ie).conjugate();
}

Eigen::Quaterniond quat_en(const double latitude, const double longitude)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxisd longitudeAngle(longitude, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd latitudeAngle(-M_PI_2 - latitude, Eigen::Vector3d::UnitY());

    return (longitudeAngle * latitudeAngle).normalized();
}

Eigen::Quaterniond quat_ne(const double latitude, const double longitude)
{
    return quat_en(latitude, longitude).conjugate();
}

Eigen::Quaterniond quat_bn(const double roll, const double pitch, const double yaw)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    // Eigen uses here a different sign convention as the physical system.
    Eigen::AngleAxisd rollAngle(-roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());

    return (rollAngle * pitchAngle * yawAngle).normalized();
}

Eigen::Quaterniond quat_nb(const double roll, const double pitch, const double yaw)
{
    return quat_bn(roll, pitch, yaw).conjugate();
}

Eigen::Quaterniond quat_bp(double mountingAngleX, double mountingAngleY, double mountingAngleZ)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd xAngle(-mountingAngleX, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yAngle(-mountingAngleY, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd zAngle(-mountingAngleZ, Eigen::Vector3d::UnitZ());

    return (xAngle * yAngle * zAngle).normalized();
}

Eigen::Quaterniond quat_pb(double mountingAngleX, double mountingAngleY, double mountingAngleZ)
{
    return quat_bp(mountingAngleX, mountingAngleY, mountingAngleZ).conjugate();
}

Eigen::Vector3d ecef2ned(const Eigen::Vector3d& position_e, const Eigen::Vector3d& latLonAlt_ref)
{
    const auto& latitude_ref = latLonAlt_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = latLonAlt_ref(1); // λ Geodetic longitude

    auto position_e_ref = lla2ecef_WGS84(latLonAlt_ref);

    Eigen::Matrix3d R_ne;
    // clang-format off
    R_ne << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(latitude_ref),
                     -std::sin(longitude_ref)                ,              std::cos(longitude_ref)             ,            0           ,
            -std::cos(latitude_ref) * std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref), -std::sin(latitude_ref);
    // clang-format on

    Eigen::Vector3d position_n = R_ne * (position_e - position_e_ref);

    return position_n;
}

Eigen::Vector3d ned2ecef(const Eigen::Vector3d& position_n, const Eigen::Vector3d& latLonAlt_ref)
{
    const auto& latitude_ref = latLonAlt_ref(0);  // 𝜙 Geodetic latitude
    const auto& longitude_ref = latLonAlt_ref(1); // λ Geodetic longitude

    auto position_e_ref = lla2ecef_WGS84(latLonAlt_ref);

    Eigen::Matrix3d R_en;
    // clang-format off
    R_en << -std::sin(latitude_ref) * std::cos(longitude_ref), -std::sin(longitude_ref), -std::cos(latitude_ref) * std::cos(longitude_ref),
            -std::sin(latitude_ref) * std::sin(longitude_ref),  std::cos(longitude_ref), -std::cos(latitude_ref) * std::sin(longitude_ref),
                         std::cos(latitude_ref)              ,             0           ,                -std::sin(latitude_ref)           ;
    // clang-format on

    Eigen::Vector3d position_e = position_e_ref + R_en * position_n;

    return position_e;
}

Eigen::Vector3d lla2ecef(const Eigen::Vector3d& latLonAlt, double a, double e_squared)
{
    const auto& latitude = latLonAlt(0);  // 𝜙 Geodetic latitude
    const auto& longitude = latLonAlt(1); // λ Geodetic longitude
    const auto& altitude = latLonAlt(2);  // Altitude (Height above ground)

    /// Radius of curvature of the ellipsoid in the prime vertical plane,
    /// i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
    double N = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    // Jekeli, 2001 (eq. 1.80) (see  Torge, 1991, for further details)
    return Eigen::Vector3d((N + altitude) * std::cos(latitude) * std::cos(longitude),
                           (N + altitude) * std::cos(latitude) * std::sin(longitude),
                           (N * (1 - e_squared) + altitude) * std::sin(latitude));
}

Eigen::Vector3d lla2ecef_WGS84(const Eigen::Vector3d& latLonAlt)
{
    return lla2ecef(latLonAlt, InsConst::WGS84_a, InsConst::WGS84_e_squared);
}

Eigen::Vector3d lla2ecef_GRS80(const Eigen::Vector3d& latLonAlt)
{
    return lla2ecef(latLonAlt, InsConst::GRS80_a, InsConst::GRS80_e_squared);
}

Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef, double a, double b, double e_squared)
{
    auto x = ecef(0);
    auto y = ecef(1);
    auto z = ecef(2);

    // Calculate longitude

    auto lon = std::atan2(y, x);

    // Start computing intermediate variables needed to compute altitude

    auto p = ecef.head(2).norm();
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

    return Eigen::Vector3d(lat, lon, alt);
}

Eigen::Vector3d ecef2lla_WGS84(const Eigen::Vector3d& ecef)
{
    return ecef2lla(ecef, InsConst::WGS84_a, InsConst::WGS84_b, InsConst::WGS84_e_squared);
}

Eigen::Vector3d ecef2lla_GRS80(const Eigen::Vector3d& ecef)
{
    return ecef2lla(ecef, InsConst::GRS80_a, InsConst::GRS80_b, InsConst::GRS80_e_squared);
}

Eigen::Vector3d sph2ecef(const Eigen::Vector3d& position_s, const double& elevation, const double& azimuth)
{
    Eigen::Matrix3d R_se;
    R_se << std::sin(elevation) * std::cos(azimuth), std::cos(elevation) * std::cos(azimuth), -std::sin(azimuth),
        std::sin(elevation) * std::sin(azimuth), std::cos(elevation) * std::sin(azimuth), std::cos(azimuth),
        std::cos(elevation), -std::sin(elevation), 0.0;

    Eigen::Vector3d position_e = R_se * position_s;

    return position_e;
}

} // namespace NAV::trafo