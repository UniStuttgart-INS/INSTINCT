#include "InsGravity.hpp"
#include "util/Logger.hpp"
#include "InsConstants.hpp"
#include "utilGravity/AssociatedLegendre.hpp"
#include "utilGravity/readAscii2Matrix.hpp"
#include "Eigen/Core"
#include <cmath>
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

Eigen::Vector3d NAV::gravity::gravity_SomiglianaAltitude(const double& latitude, const double& altitude)
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

    // Magnitude of the gravity, i.e. without orientation
    double gravityMagnitude = k * g_0;

    Eigen::Vector3d gravity_n = centrifugalAcceleration_WGS84(latitude, altitude, gravityMagnitude);

    return gravity_n;
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
}

Eigen::Vector3d NAV::gravity::gravity_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0));

    // Magnitude of the gravity, i.e. without orientation
    double gravityMagnitude = InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
                              - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(sin(latitudeGeocentric), 2.0) - 1)
                              - std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    Eigen::Vector3d gravity_n = centrifugalAcceleration_WGS84(latitude, altitude, gravityMagnitude);

    return gravity_n;
}

Eigen::Vector3d NAV::gravity::centrifugalAcceleration_WGS84(const double& latitude, const double& altitude, double gravityMagnitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0));
    double radiusEarthBody = radiusSpheroid + std::abs(altitude);

    // Centrifugal acceleration magnitude on earth's surface and on body
    double centrifugalAccelerationMagnitudeSurface = std::pow(InsConst::angularVelocity_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);
    double centrifugalAccelerationMagnitudeBody = std::pow(InsConst::angularVelocity_ie, 2.0) * radiusEarthBody * std::pow(std::cos(latitudeGeocentric), 2.0);

    // North-component of the centrifugal acceleration
    double centrifAcc_nS = centrifugalAccelerationMagnitudeSurface * std::sin(latitude);
    double centrifAcc_nB = centrifugalAccelerationMagnitudeBody * std::sin(latitude);
    // Down-component of the centrifugal acceleration
    double centrifAcc_dS = centrifugalAccelerationMagnitudeSurface * std::cos(latitude);
    double centrifAcc_dB = centrifugalAccelerationMagnitudeBody * std::cos(latitude);

    // Down component of the gravitation (i.e. NOT gravity! See Groves (2013) Chapter 2.4.7) on earth's surface
    auto gravitation_down = (2.0 * centrifAcc_dS + std::sqrt(4.0 * centrifAcc_dS * centrifAcc_dS - 4.0 * (centrifAcc_nS * centrifAcc_nS + centrifAcc_dS * centrifAcc_dS - gravityMagnitude * gravityMagnitude))) / 2.0;

    // Altitude compensation (see Groves (2013) Chapter 2.4.7 - eq. 2.138)
    auto gravitation_downB = std::pow(radiusSpheroid / radiusEarthBody, 2.0) * gravitation_down;

    // Gravity vector in NED
    Eigen::Vector3d gravity_n(-centrifAcc_nB, 0.0, gravitation_downB - centrifAcc_dB);

    return gravity_n;
}

Eigen::Vector3d NAV::gravity::gravity_EGM96(const double& latitude, const double& longitude, const double& altitude, int ndegree)
{
    // Read ascii file that contains all EGM96 parameters
    static Eigen::MatrixXd coeffs = NAV::utilGravity::readAscii2Matrix();

    // Geocentric latitude determination from geographic latitude and elevation and azimuth
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    double elevation = M_PI_2 - latitudeGeocentric; // radians
    double azimuth = longitude;

    // Radius determination (Spheroid)
    double radius = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    /// g_n Gravity vector in [m/s^2], in spherical coordinates
    Eigen::Vector3d gravity_sph = { -InsConst::WGS84_MU / (radius * radius), 0.0, 0.0 };

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    auto [P, Pd] = NAV::utilGravity::associatedLegendre(ndegree + 1, std::sin(elevation));

    double Pnm = 0;
    double Pnmd = 0;

    auto coeffsRows = coeffs.rows();

    for (int i = 0; i < coeffsRows; i++)
    {
        // Retrieving EGM96 coefficients
        auto n = static_cast<int>(coeffs(i, 0)); // Degree of the Associated Legendre Polynomial
        auto m = static_cast<int>(coeffs(i, 1)); // Order of the Associated Legendre Polynomial
        auto C = coeffs(i, 2);
        auto S = coeffs(i, 3);

        if (n == ndegree + 1)
        {
            // Ending of the for-loop once the iterated 'n' becomes larger than the user-defined 'ndegree'
            i = static_cast<int>(coeffsRows);
        }
        else
        {
            // Retrieving the parameters of the Associated Legendre Polynomials
            Pnm = P(m, n);
            Pnmd = Pd(m, n);

            auto nd = static_cast<double>(n);
            auto md = static_cast<double>(m);

            // Setting the gravity vector dependent on Position(spherical) and EGM96 coefficients
            gravity_sph(0) = gravity_sph(0) - std::pow(InsConst::WGS84_a, nd) * (nd + 1.0) * InsConst::WGS84_MU * Pnm * (C * cos(md * azimuth) + S * sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)));
            gravity_sph(1) = gravity_sph(1) + std::pow(InsConst::WGS84_a, nd) * InsConst::WGS84_MU * Pnmd * std::cos(elevation) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)));
            gravity_sph(2) = gravity_sph(2) + std::pow(InsConst::WGS84_a, nd) * InsConst::WGS84_MU * md * Pnm * (S * std::cos(md * azimuth) - C * std::sin(md * azimuth)) / (std::pow(radius, (nd + 2.0)) * std::sin(elevation));
        }
    }

    // Rotation of the spherical gravity vector to ECEF coordinates
    Eigen::Vector3d eleAziAlt = { 0.0, elevation, azimuth };
    Eigen::Vector3d gravity_ecef = NAV::trafo::sph2ecef(gravity_sph, eleAziAlt);

    // Rotation of the ECEF gravity vector to NED
    Eigen::Matrix3d R_ne;
    R_ne << -std::sin(latitude) * std::cos(azimuth), -std::sin(latitude) * std::sin(azimuth), std::cos(latitude),
        -std::sin(azimuth), std::cos(azimuth), 0.0,
        -std::cos(latitude) * std::cos(azimuth), -std::cos(latitude) * std::sin(azimuth), -std::sin(latitude);

    // trafo::quat_ne(latitude, azimuth);

    Eigen::Vector3d gravity_n = R_ne * gravity_ecef;

    return gravity_n;
}
