#include "InsGravity.hpp"

#include "InsConstants.hpp"

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
