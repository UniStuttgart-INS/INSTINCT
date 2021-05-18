#include "InsGravity.hpp"

#include "InsConstants.hpp"
#include <cmath>
//#include <vector>

double NAV::gravity::gravityMagnitude_SomiglianaAltitude(const double& latitude, const double& altitude)
{
    // eq 6.16 has a fault in the denominator, it should be a sin^2(latitude)
    double g_0 = 9.7803253359 * (1.0 + 1.931853e-3 * std::pow(std::sin(latitude), 2))
                 / std::sqrt(1.0 - InsConst::WGS84_e_squared * std::pow(std::sin(latitude), 2));

    // Altitude compensation (Matlab example from Chapter 6_GNSS_INS_1 - glocal.m)
    double k = 1
               - (2 * altitude / InsConst::WGS84_a)
                     * (1 + InsConst::WGS84_f
                        + (std::pow(InsConst::angularVelocity_ie * InsConst::WGS84_a, 2))
                              * (InsConst::WGS84_b / InsConst::WGS84_MU))
               + 3 * std::pow(altitude / InsConst::WGS84_a, 2);

    return k * g_0;
}

double NAV::gravity::gravityMagnitude_WGS84_Skydel(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitude_geocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 ellipsoid plus local altitude --> possible error!! altitude is in n-sys, while WGS84 is actually e-sys!
    double radius_effective = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitude_geocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    return InsConst::WGS84_MU * std::pow(radius_effective, -2.0)
           - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radius_effective, -4.0) * (3 * std::pow(sin(latitude_geocentric), 2.0) - 1)
           - std::pow(InsConst::angularVelocity_ie, 2.0) * radius_effective * std::pow(std::cos(latitude_geocentric), 2.0);
    // Gravity vector in n system
    //const Eigen::Vector3d gravityWGS84(centrifugation, 0, gravitation - centrifugation);
}

double NAV::gravity::gravityMagnitude_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitude_geocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // Radius of spheroid determination (assuming a locally flat earth)
    double radius_effective = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitude_geocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after radius of spheroid (gravity = gravitation - centrifucalAcceleration)
    return InsConst::WGS84_MU * std::pow(radius_effective, -2.0)
           - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radius_effective, -4.0) * (3 * std::pow(sin(latitude_geocentric), 2.0) - 1)
           - std::pow(InsConst::angularVelocity_ie, 2.0) * radius_effective * std::pow(std::cos(latitude_geocentric), 2.0);
}

double NAV::gravity::centrifugalAcc_WGS84(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitude_geocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 ellipsoid plus local altitude
    double radius_effective = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::sin(latitude_geocentric)) - altitude;

    // Centrifugal force in e system
    //double centrifugation = -std::pow(InsConst::angularVelocity_ie, 2.0) * radius_effective * std::pow(std::cos(latitude_geocentric), 2.0);
    return -std::pow(InsConst::angularVelocity_ie, 2.0) * radius_effective * std::cos(latitude_geocentric);
}
