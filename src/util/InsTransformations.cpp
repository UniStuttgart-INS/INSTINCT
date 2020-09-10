#include "InsTransformations.hpp"

namespace NAV::trafo
{
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
    return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Quaterniond quat_i2e(const double time, const double angularRate_ie)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd zAngle(angularRate_ie * time, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q(zAngle);
    return q;
}

Eigen::Matrix3d DCM_i2e(const double time, const double angularRate_ie)
{
    // Eigen::Matrix3d dcm;
    // // clang-format off
    // dcm <<  std::cos(angularRate_ie * time), std::sin(angularRate_ie * time), 0,
    //        -std::sin(angularRate_ie * time), std::cos(angularRate_ie * time), 0,
    //                       0                ,               0                , 1;
    // // clang-format on
    // return dcm;

    return quat_i2e(time, angularRate_ie).toRotationMatrix();
}

Eigen::Quaterniond quat_n2e(const double latitude, const double longitude)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd longitudeAngle(-longitude, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd latitudeAngle(M_PI_2 + latitude, Eigen::Vector3d::UnitY());

    Eigen::Quaterniond q = longitudeAngle * latitudeAngle;
    return q;
}

Eigen::Matrix3d DCM_n2e(const double latitude, const double longitude)
{
    // Eigen::Matrix3d dcm;
    // // clang-format off
    // dcm << -std::sin(latitude) * std::cos(longitude), -std::sin(longitude), -std::cos(latitude) * std::cos(longitude),
    //        -std::sin(latitude) * std::sin(longitude),  std::cos(longitude), -std::cos(latitude) * std::sin(longitude),
    //                   std::cos(latitude)            ,           0         ,           -std::sin(latitude)            ;
    // // clang-format on
    // return dcm;

    return quat_n2e(latitude, longitude).toRotationMatrix();
}

Eigen::Quaterniond quat_b2n(const double roll, const double pitch, const double yaw)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd rollAngle(-roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Matrix3d DCM_b2n(const double roll, const double pitch, const double yaw)
{
    // const double& R = roll;
    // const double& P = pitch;
    // const double& Y = yaw;

    // Eigen::Matrix3d dcm;
    // // clang-format off
    // dcm << std::cos(Y)*std::cos(P), std::cos(Y)*std::sin(P)*std::sin(R) - std::sin(Y)*std::cos(R), std::cos(Y)*std::sin(P)*std::cos(R) + std::sin(Y)*std::sin(R),
    //        std::sin(Y)*std::cos(P), std::sin(Y)*std::sin(P)*std::sin(R) + std::cos(Y)*std::cos(R), std::sin(Y)*std::sin(P)*std::cos(R) - std::cos(Y)*std::sin(R),
    //             -std::sin(P)      ,                   std::cos(P)*std::sin(R)                    ,                   std::cos(P)*std::cos(R)                    ;
    // // clang-format on
    // return dcm;

    return quat_b2n(roll, pitch, yaw).toRotationMatrix();
}

Eigen::Vector3d llh2ecef_wgs84(const double latitude, const double longitude, const double height)
{
    return Eigen::Vector3d(latitude, longitude, height);
}

} // namespace NAV::trafo