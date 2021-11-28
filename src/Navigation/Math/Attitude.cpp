#include "Attitude.hpp"

#include <cmath>

#include "Math.hpp"

namespace NAV::math
{

double rollFromStaticAccelerationObs(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72a)
    return sgn(accel_b.z()) * std::asin(accel_b.y() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2(accel_b.y(), accel_b.z());
}

double pitchFromStaticAccelerationObs(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72b)
    return -sgn(accel_b.z()) * std::asin(accel_b.x() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2((-accel_b.x()), sqrt(std::pow(accel_b.y(), 2) + std::pow(accel_b.z(), 2)));
}

double yawFromVelocity(const Eigen::Vector3d& velocity_n)
{
    return std::atan2(velocity_n(1), velocity_n(0));
}

double pitchFromVelocity(const Eigen::Vector3d& velocity_n)
{
    return std::atan(-velocity_n(2) / std::sqrt(std::pow(velocity_n(0), 2) + std::pow(velocity_n(1), 2)));
}

} // namespace NAV::math