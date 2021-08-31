#include "InsGravity.hpp"

#include <cmath>
#include <optional>

#include "Eigen/Core"
#include "util/InsTransformations.hpp"
#include "util/Logger.hpp"
#include "InsConstants.hpp"
#include "Gravity/AssociatedLegendre.hpp"
#include "Gravity/egm96Coeffs.hpp"

#include <string>
#include <fstream>
#include <streambuf>

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

Eigen::Vector3d NAV::gravity::gravity_SomiglianaAltitude(const double& latitude, const double& altitude)
{
    Eigen::Vector3d gravity_n(0.0, 0.0, gravityMagnitude_SomiglianaAltitude(latitude, altitude));

    return gravity_n;
}

double NAV::gravity::gravityMagnitude_WGS84_Skydel(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 spheroid plus local altitude --> possible error!! altitude in lla should be added rather than subtracted!
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    return InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
           - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
           - std::pow(InsConst::angularVelocity_ie_Skydel, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
}

Eigen::Vector3d NAV::gravity::gravity_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Magnitude of the gravity, i.e. without orientation
    double gravityMagnitude = InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
                              - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                              - std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    Eigen::Vector3d gravity_n(0.0, 0.0, gravityMagnitude);

    return gravity_n;
}

Eigen::Vector3d NAV::gravity::gravity_EGM96(const double& latitude, const double& longitude, const double& altitude, int ndegree)
{
    Eigen::Vector3d pos_lla(latitude, longitude, altitude);
    Eigen::Vector3d pos_ecef = trafo::lla2ecef_WGS84(pos_lla);

    // Geocentric latitude determination from Groves (2013) - eq. (2.114)
    double latitudeGeocentric = std::atan(pos_ecef(2) / std::sqrt(pos_ecef(0) * pos_ecef(0) + pos_ecef(1) * pos_ecef(1)));

    // Spherical coordinates
    double radius = std::sqrt(pos_ecef(0) * pos_ecef(0) + pos_ecef(1) * pos_ecef(1) + pos_ecef(2) * pos_ecef(2));
    double elevation = M_PI_2 - latitudeGeocentric; // [rad]
    double azimuth = longitude;

    /// Gravity vector in [m/s^2], in n-system
    Eigen::Vector3d gravity_n = { 0.0, 0.0, 0.0 };

    double Pnm = 0;
    double Pnmd = 0;

    auto coeffsRows = egm96Coeffs.size();

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    auto [P, Pd] = NAV::util::gravity::associatedLegendre(ndegree, elevation);
    LOG_DATA("NEW Associated Legendre Polynomial coefficients: P_new =\n{}\nPd_new =\n{}", P, Pd);

    for (size_t i = 0; i < coeffsRows; i++) // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult) // FIXME: Wrong error message about Eigen (error: The left operand of '*' is a garbage value)
    {
        // Retrieving EGM96 coefficients
        auto n = static_cast<int>(egm96Coeffs.at(i).at(0)); // Degree of the Associated Legendre Polynomial
        auto m = static_cast<int>(egm96Coeffs.at(i).at(1)); // Order of the Associated Legendre Polynomial
        auto C = egm96Coeffs.at(i).at(2);
        auto S = egm96Coeffs.at(i).at(3);

        if (n == ndegree + 1)
        {
            // Ending of the for-loop once the iterated 'n' becomes larger than the user-defined 'ndegree'
            i = coeffsRows;
        }
        else
        {
            // Retrieving the parameters of the associated Legendre Polynomials
            Pnm = P(n, m);
            Pnmd = Pd(n, m);

            auto nd = static_cast<double>(n);
            auto md = static_cast<double>(m);

            // Gravity vector from differentiation of the gravity potential in spherical coordinates (see 'GUT User Guide' eq. 7.4.2) - centrifugal acceleration added below
            gravity_n(0) += std::pow((InsConst::WGS84_a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnmd;
            gravity_n(1) += std::pow((InsConst::WGS84_a / radius), nd) * md * (C * std::sin(md * azimuth) - S * std::cos(md * azimuth)) * Pnm;
            gravity_n(2) += (nd + 1.0) * std::pow((InsConst::WGS84_a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnm;
        }
    }

    gravity_n(0) = -InsConst::WGS84_MU / (radius * radius) * gravity_n(0);
    gravity_n(1) = (1.0 / std::sin(elevation)) * (-InsConst::WGS84_MU / (radius * radius)) * gravity_n(1);
    gravity_n(2) = InsConst::WGS84_MU / (radius * radius) * (1.0 + gravity_n(2));

    return gravity_n;
}

Eigen::Vector3d NAV::gravity::centrifugalAcceleration(const double& latitude, const double& altitude, Eigen::Vector3d& acceleration)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    double radiusEarthBody = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + std::abs(altitude);

    // Centrifugal acceleration - components North and Down (see 'GUT User Guide' eq. 7.4.2)
    double centrifugalN = InsConst::angularVelocity_ie * InsConst::angularVelocity_ie * radiusEarthBody * std::sin(M_PI_2 - latitudeGeocentric) * std::cos(M_PI_2 - latitudeGeocentric);
    double centrifugalD = InsConst::angularVelocity_ie * InsConst::angularVelocity_ie * radiusEarthBody * std::sin(M_PI_2 - latitudeGeocentric) * std::sin(M_PI_2 - latitudeGeocentric);

    // Acceleration vector in NED
    Eigen::Vector3d acceleration_n(acceleration(0) - centrifugalN, acceleration(1), acceleration(2) - centrifugalD);

    return acceleration_n;
}
