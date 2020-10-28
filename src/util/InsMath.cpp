#include "InsMath.hpp"

#include "gcem.hpp"

double NAV::measureDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = InsConst::WGS84_a;
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::pow(std::sin(dLat / 2.0), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double d = R * c;
    return d; // meters
}

double NAV::rollFromStaticAccelerationObs(const NAV::Vector3d<NAV::Body>& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72a)
    return gcem::sgn(accel_b.z()) * std::asin(accel_b.y() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2(accel_b.y(), accel_b.z());
}

double NAV::pitchFromStaticAccelerationObs(const NAV::Vector3d<NAV::Body>& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72b)
    return -gcem::sgn(accel_b.z()) * std::asin(accel_b.x() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2((-accel_b.x()), sqrt(std::pow(accel_b.y(), 2) + std::pow(accel_b.z(), 2)));
}