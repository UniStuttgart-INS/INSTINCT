#include "InsUtil.hpp"

namespace NAV
{
Eigen::Quaterniond euler2quaternion(const double yaw, const double pitch, const double roll)
{
    // Initialize angle-axis rotation from an angle in radian and an axis which must be normalized.
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

} // namespace NAV