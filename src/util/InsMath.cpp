#include "InsMath.hpp"

#include "gcem.hpp"

#include "InsMechanization.hpp"

double NAV::calcGreatCircleDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = geocentricRadius(lat1, earthRadius_E(lat1));
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::pow(std::sin(dLat / 2.0), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double d = R * c;
    return d; // meters
}

double NAV::calcGeographicalDistance(double lat1, double lon1, double lat2, double lon2)
{
    if (lat1 == lat2 && lon1 == lon2)
    {
        return 0;
    }
    // First convert the latitudes 𝜙₁,𝜙₂ of the two points to reduced latitudes 𝛽₁,𝛽₂
    double beta1 = std::atan((1 - InsConst::WGS84_f) * std::tan(lat1));
    double beta2 = std::atan((1 - InsConst::WGS84_f) * std::tan(lat2));

    // Then calculate the central angle 𝜎 in radians between two points 𝛽₁,𝜆₁ and 𝛽₂,𝜆₂ on a sphere using the
    // Great-circle distance method (law of cosines or haversine formula), with longitudes 𝜆₁ and 𝜆₂ being the same on the sphere as on the spheroid.
    double sigma = calcGreatCircleDistance(beta1, lon1, beta2, lon2)
                   / geocentricRadius(lat1, earthRadius_E(lat1));

    double P = (beta1 + beta2) / 2;
    double Q = (beta2 - beta1) / 2;

    double X = (sigma - sin(sigma)) * std::pow((std::sin(P) * std::cos(Q)) / std::cos(sigma / 2), 2);
    double Y = (sigma + sin(sigma)) * std::pow((std::cos(P) * std::sin(Q)) / std::sin(sigma / 2), 2);

    return InsConst::WGS84_a * (sigma - InsConst::WGS84_f / 2.0 * (X + Y));
}

double NAV::rollFromStaticAccelerationObs(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72a)
    return gcem::sgn(accel_b.z()) * std::asin(accel_b.y() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2(accel_b.y(), accel_b.z());
}

double NAV::pitchFromStaticAccelerationObs(const Eigen::Vector3d& accel_b)
{
    // See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6 - eq. 2.72b)
    return -gcem::sgn(accel_b.z()) * std::asin(accel_b.x() / accel_b.norm());

    // Another possible calculation would be:
    // return std::atan2((-accel_b.x()), sqrt(std::pow(accel_b.y(), 2) + std::pow(accel_b.z(), 2)));
}

double NAV::yawFromVelocity(const Eigen::Vector3d& velocity_n)
{
    return std::atan2(velocity_n(1), velocity_n(0));
}

double NAV::pitchFromVelocity(const Eigen::Vector3d& velocity_n)
{
    return std::atan(-velocity_n(2) / std::sqrt(std::pow(velocity_n(0), 2) + std::pow(velocity_n(1), 2)));
}
