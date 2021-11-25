#include "InsMath.hpp"

#include "gcem.hpp"

#include "InsMechanization.hpp"

double NAV::calcGreatCircleDistance(double lat1, double lon1, double lat2, double lon2)
{
    double R = InsConst::WGS84_a;
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

uint64_t NAV::factorial(uint64_t n)
{
    // uint64_t is required to calculate factorials of n > 12 (Limit of uint32_t). The limit of uint64_t is at n = 20
    constexpr std::array factorials = {
        uint64_t(1),                   // 0
        uint64_t(1),                   // 1
        uint64_t(2),                   // 2
        uint64_t(6),                   // 3
        uint64_t(24),                  // 4
        uint64_t(120),                 // 5
        uint64_t(720),                 // 6
        uint64_t(5040),                // 7
        uint64_t(40320),               // 8
        uint64_t(362880),              // 9
        uint64_t(3628800),             // 10
        uint64_t(39916800),            // 11
        uint64_t(479001600),           // 12
        uint64_t(6227020800),          // 13
        uint64_t(87178291200),         // 14
        uint64_t(1307674368000),       // 15
        uint64_t(20922789888000),      // 16
        uint64_t(355687428096000),     // 17
        uint64_t(6402373705728000),    // 18
        uint64_t(121645100408832000),  // 19
        uint64_t(2432902008176640000), // 20
    };

    if (n < factorials.size())
    {
        return factorials.at(n);
    }
    return n * factorial(n - 1);
}