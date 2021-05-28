#include "InsGravity.hpp"
#include "util/Logger.hpp"
#include "InsConstants.hpp"
#include "utilGravity/AssociatedLegendre.hpp"
#include "utilGravity/readAscii2Matrix.hpp"
// #include "Eigen/Dense"
#include "Eigen/Core"
#include <cmath>
//#include <vector>
#include "util/InsTransformations.hpp"

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

double NAV::gravity::gravityMagnitude_WGS84_Skydel(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 spheroid plus local altitude --> possible error!! altitude in lla should be added rather than subtracted!
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    return InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
           - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(sin(latitudeGeocentric), 2.0) - 1)
           - std::pow(InsConst::angularVelocity_ie_Skydel, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
    // Gravity vector in n system
    //const Eigen::Vector3d gravityWGS84(centrifugation, 0, gravitation - centrifugation);
}

double NAV::gravity::gravityMagnitude_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Derivation of gravity, i.e. gravitational potential derived after radius of spheroid (gravity = gravitation - centrifucalAcceleration)
    return InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
           - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(sin(latitudeGeocentric), 2.0) - 1)
           - std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
}

double NAV::gravity::centrifugalAccelerationMagnitude_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Centrifugal force in e system
    //double centrifugation = -std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
    return std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
}

Eigen::Vector3d NAV::gravity::gravity_EGM96(const double& latitude, const double& altitude, int ndegree)
{
    // Read ascii file that contains all EGM96 parameters
    Eigen::MatrixXd coeffs = NAV::utilGravity::readAscii2Matrix();

    // Geocentric latitude determination from geographic latitude and elevation and azimuth
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    double elevation = M_PI_2 - latitudeGeocentric; // radians
    double azimuth = 0;                             // mmm dummy value!

    // Radius determination (Spheroid)
    double radius = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Mean gravity potential (dividing again by radius would --> 9.81 m/s²)
    // double u0 = InsConst::WGS84_MU / radius;

    /// g_n Gravity vector in [m/s^2], in spherical coordinates
    Eigen::Vector3d gravity_sph = { -InsConst::WGS84_MU / (radius * radius), 0.0, 0.0 };

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    // auto ndegree = 10; //static_cast<uint16_t>(coeffs.maxCoeff());
    auto [P, Pd] = NAV::utilGravity::associatedLegendre(ndegree + 1, std::sin(elevation));

    double Pnm = 0;
    double Pnmd = 0;

    auto coeffsRows = coeffs.rows();
    // auto coeffsCols = coeffs.cols();

    for (int i = 0; i < coeffsRows; i++)
    {
        auto n = static_cast<int>(coeffs(i, 0));
        auto m = static_cast<int>(coeffs(i, 1));
        auto C = coeffs(i, 2);
        auto S = coeffs(i, 3);

        if (n == ndegree + 1)
        {
            // Ending of the for-loop once the iterated 'n' becomes larger than the user-defined 'ndegree'
            i = static_cast<int>(coeffsRows);
        }
        else
        {
            // LOG_DEBUG("Setting Pnm Matrix Elements at n = {} and m = {}", n, m);
            Pnm = P(m, n);   // n - 2);
            Pnmd = Pd(m, n); // - 2);

            //u = u0 + std::pow(InsConst::WGS84_a, n) * InsConst::WGS84_MU * (Pnm * (C * std::cos(static_cast<double>(m) * azimuth) + S * std::sin(static_cast<double>(m) * azimuth))) / (std::pow(radius, (static_cast<double>(n + 1))));

            auto nd = static_cast<double>(n);
            auto md = static_cast<double>(m);
            // LOG_DEBUG("Setting Gravity vector");
            gravity_sph(0) = gravity_sph(0) - std::pow(InsConst::WGS84_a, nd) * (nd + 1.0) * InsConst::WGS84_MU * Pnm * (C * cos(md * azimuth) + S * sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)));
            gravity_sph(1) = gravity_sph(1) + std::pow(InsConst::WGS84_a, nd) * InsConst::WGS84_MU * Pnmd * std::cos(elevation) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)));
            gravity_sph(2) = gravity_sph(2) + std::pow(InsConst::WGS84_a, nd) * InsConst::WGS84_MU * md * Pnm * (S * std::cos(md * azimuth) - C * std::sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)) * std::sin(elevation));
        }
    }

    Eigen::Matrix3d sph2ecef;
    sph2ecef << std::sin(elevation) * std::cos(azimuth), std::cos(elevation) * std::cos(azimuth), -std::sin(azimuth),
        std::sin(elevation) * std::sin(azimuth), std::cos(elevation) * std::sin(azimuth), std::cos(azimuth),
        std::cos(elevation), -std::sin(elevation), 0.0;
    // rot = [sin(el)*cos(az), cos(el)*cos(az), -sin(az);
    //    sin(el)*sin(az), cos(el)*sin(az), cos(az);
    //    cos(el),         -sin(el),        0];

    Eigen::Vector3d gravity_ecef = sph2ecef * gravity_sph;

    // const auto& latitude_ref = latLonAlt_ref(0);  // 𝜙 Geodetic latitude
    //     const auto& longitude_ref = latLonAlt_ref(1); // λ Geodetic longitude

    Eigen::Matrix3d R_ne;
    R_ne << -std::sin(latitude) * std::cos(azimuth), -std::sin(latitude) * std::sin(azimuth), std::cos(latitude),
        -std::sin(azimuth), std::cos(azimuth), 0.0,
        -std::cos(latitude) * std::cos(azimuth), -std::cos(latitude) * std::sin(azimuth), -std::sin(latitude);

    // trafo::quat_ne(latitude, azimuth);

    Eigen::Vector3d gravity_n = R_ne * gravity_ecef;

    return gravity_n;
}