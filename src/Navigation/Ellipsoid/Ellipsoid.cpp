#include "Ellipsoid.hpp"

#include <cmath>

namespace NAV::ellipsoid
{

double calcGreatCircleDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = geocentricRadius(lat1, earthRadius_E(lat1));
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::pow(std::sin(dLat / 2.0), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double d = R * c;
    return d; // meters
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
                   / geocentricRadius(lat1, earthRadius_E(lat1));

    double P = (beta1 + beta2) / 2;
    double Q = (beta2 - beta1) / 2;

    double X = (sigma - std::sin(sigma)) * std::pow((std::sin(P) * std::cos(Q)) / std::cos(sigma / 2), 2);
    double Y = (sigma + std::sin(sigma)) * std::pow((std::cos(P) * std::sin(Q)) / std::sin(sigma / 2), 2);

    return InsConst::WGS84_a * (sigma - InsConst::WGS84_f / 2.0 * (X + Y));
}

double earthRadius_N(const double& latitude, const double& a, const double& e_squared)
{
    double k = std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    /// North/South (meridian) earth radius [m]
    double R_N = a * (1 - e_squared) / std::pow(k, 3);

    return R_N;
}

double earthRadius_E(const double& latitude, const double& a, const double& e_squared)
{
    /// East/West (prime vertical) earth radius [m]
    double R_E = a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    return R_E;
}

Eigen::Vector3d transportRate(const Eigen::Vector3d& latLonAlt__t1,  // [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
                              const Eigen::Vector3d& velocity_n__t1, // v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
                              const double& R_N,                     // R_N North/South (meridian) earth radius [m]
                              const double& R_E)                     // R_E East/West (prime vertical) earth radius [m]
{
    /// 𝜙 Latitude in [rad]
    const auto& latitude = latLonAlt__t1(0);
    /// h Altitude in [m]
    const auto& altitude = latLonAlt__t1(2);

    /// Velocity North in [m/s]
    const auto& v_N = velocity_n__t1(0);
    /// Velocity East in [m/s]
    const auto& v_E = velocity_n__t1(1);

    /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame,
    /// in navigation coordinates see Gleason (eq. 6.15)
    Eigen::Vector3d angularVelocity_en_n__t1;
    angularVelocity_en_n__t1(0) = v_E / (R_E + altitude);
    angularVelocity_en_n__t1(1) = -v_N / (R_N + altitude);
    angularVelocity_en_n__t1(2) = -angularVelocity_en_n__t1(0) * std::tan(latitude);

    return angularVelocity_en_n__t1;
}

} // namespace NAV::ellipsoid
