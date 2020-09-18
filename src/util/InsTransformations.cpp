#include "InsTransformations.hpp"

#include "util/Logger.hpp"

Eigen::Vector3d NAV::trafo::deg2rad3(const Eigen::Vector3d& deg)
{
    return deg * M_PI / 180.0;
}

Eigen::Vector3d NAV::trafo::rad2deg3(const Eigen::Vector3d& rad)
{
    return rad * 180.0 / M_PI;
}

Eigen::Vector3d NAV::trafo::quat2eulerZYX(const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Quaterniond NAV::trafo::quat_i2e(const double time, const double angularRate_ie)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd zAngle(angularRate_ie * time, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q(zAngle);
    return q;
}

Eigen::Quaterniond NAV::trafo::quat_n2e(const double latitude, const double longitude)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd longitudeAngle(-longitude, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd latitudeAngle(M_PI_2 + latitude, Eigen::Vector3d::UnitY());

    Eigen::Quaterniond q = longitudeAngle * latitudeAngle;
    return q;
}

Eigen::Quaterniond NAV::trafo::quat_b2n(const double roll, const double pitch, const double yaw)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Quaterniond NAV::trafo::quat_p2b(double mountingAngleX, double mountingAngleY, double mountingAngleZ)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd xAngle(-mountingAngleX, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yAngle(-mountingAngleY, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd zAngle(-mountingAngleZ, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = zAngle * yAngle * xAngle;
    return q;
}

Eigen::Vector3d NAV::trafo::llh2ecef(const double latitude, const double longitude, const double height, double a, double e_squared)
{
    /// Radius of curvature of the ellipsoid in the prime vertical plane,
    /// i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
    double N = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    // Jekeli, 2001 (eq. 1.80) (see  Torge, 1991, for further details)
    return Eigen::Vector3d((N + height) * std::cos(latitude) * std::cos(longitude),
                           (N + height) * std::cos(latitude) * std::sin(longitude),
                           (N * (1 - e_squared) + height) * std::sin(latitude));
}

Eigen::Vector3d NAV::trafo::llh2ecef_WGS84(const double latitude, const double longitude, const double height)
{
    return llh2ecef(latitude, longitude, height, InsConst::WGS84_a, InsConst::WGS84_e_squared);
}

Eigen::Vector3d NAV::trafo::llh2ecef_GRS80(const double latitude, const double longitude, const double height)
{
    return llh2ecef(latitude, longitude, height, InsConst::GRS80_a, InsConst::GRS80_e_squared);
}

Eigen::Vector3d NAV::trafo::ecef2llh(const Eigen::Vector3d& ecef, double a, double e_squared)
{
    // Value is used every iteration and does not change
    double sqrt_x1x1_x2x2 = std::sqrt(std::pow(ecef(0), 2) + std::pow(ecef(1), 2));

    // Latitude with initial assumption that h = 0 (eq. 1.85)
    double latitude = std::atan2(ecef(2) / (1 - e_squared), sqrt_x1x1_x2x2);

    double N{};
    size_t maxIterationCount = 6;
    for (size_t i = 0; i < maxIterationCount; i++) // Convergence should break the loop, but better limit the loop itself
    {
        // Radius of curvature of the ellipsoid in the prime vertical plane,
        // i.e., the plane containing the normal at P and perpendicular to the meridian (eq. 1.81)
        N = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

        // Latitude (eq. 1.84)
        double newLatitude = std::atan2(ecef(2) + e_squared * N * std::sin(latitude), sqrt_x1x1_x2x2);

        // Check convergence
        if (std::abs(newLatitude - latitude) <= 0.001)
        {
            latitude = newLatitude;
            break;
        }

        if (i == maxIterationCount - 1)
        {
            LOG_WARN("ECEF2LLH conversion did not converge! Difference is still at {} [rad]", std::abs(newLatitude - latitude));
        }

        latitude = newLatitude;
    }

    // Longitude (eq. 1.84)
    double longitude = std::atan2(ecef(1), ecef(0));
    // Height (eq. 1.84)
    double height = sqrt_x1x1_x2x2 / std::cos(latitude);
    height -= N;

    return Eigen::Vector3d(latitude, longitude, height);
}

Eigen::Vector3d NAV::trafo::ecef2llh_WGS84(const Eigen::Vector3d& ecef)
{
    return ecef2llh(ecef, InsConst::WGS84_a, InsConst::WGS84_e_squared);
}

Eigen::Vector3d NAV::trafo::ecef2llh_GRS80(const Eigen::Vector3d& ecef)
{
    return ecef2llh(ecef, InsConst::GRS80_a, InsConst::GRS80_e_squared);
}