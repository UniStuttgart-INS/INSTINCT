#include "Ellipsoid.hpp"

#include <cmath>

namespace NAV
{

double calcGreatCircleDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = calcGeocentricRadius(lat1, calcEarthRadius_E(lat1));
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::pow(std::sin(dLat / 2.0), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c; // meters
}

double calcGeographicalDistance(double lat1, double lon1, double lat2, double lon2)
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
                   / calcGeocentricRadius(lat1, calcEarthRadius_E(lat1));

    double P = (beta1 + beta2) / 2;
    double Q = (beta2 - beta1) / 2;

    double X = (sigma - std::sin(sigma)) * std::pow((std::sin(P) * std::cos(Q)) / std::cos(sigma / 2), 2);
    double Y = (sigma + std::sin(sigma)) * std::pow((std::cos(P) * std::sin(Q)) / std::sin(sigma / 2), 2);

    return InsConst::WGS84_a * (sigma - InsConst::WGS84_f / 2.0 * (X + Y));
}

double calcEarthRadius_N(const double& latitude, const double& a, const double& e_squared)
{
    double k = std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    // North/South (meridian) earth radius [m]
    return a * (1 - e_squared) / std::pow(k, 3);
}

double calcEarthRadius_E(const double& latitude, const double& a, const double& e_squared)
{
    // East/West (prime vertical) earth radius [m]
    return a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));
}

double calcGeocentricRadius(const double& latitude, const double& R_E, const double& e_squared)
{
    return R_E * std::sqrt(std::pow(std::cos(latitude), 2) + std::pow((1.0 - e_squared) * std::sin(latitude), 2));
}

} // namespace NAV
