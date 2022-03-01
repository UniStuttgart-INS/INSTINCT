#include "Gravity.hpp"

#include <cmath>
#include <optional>

#include "Eigen/Core"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"
#include "Navigation/Constants.hpp"
#include "internal/AssociatedLegendre.hpp"
#include "internal/egm96Coeffs.hpp"

#include <string>
#include <fstream>
#include <streambuf>

namespace NAV
{

const char* to_string(GravitationModel gravitationModel)
{
    switch (gravitationModel)
    {
    case GravitationModel::WGS84:
        return "WGS84";
    case GravitationModel::WGS84_Skydel:
        return "WGS84 (Skydel Constants)";
    case GravitationModel::Somigliana:
        return "Somigliana";
    case GravitationModel::EGM96:
        return "EGM96";
    case GravitationModel::OFF:
        return "OFF";
    case GravitationModel::COUNT:
        return "";
    }
    return "";
}

Eigen::Vector3d n_calcGravitation(const Eigen::Vector3d& lla_position, GravitationModel gravitationModel)
{
    const double& latitude = lla_position(0);
    const double& altitude = lla_position(2);

    if (gravitationModel == GravitationModel::WGS84)
    {
        return n_calcGravitation_WGS84(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::WGS84_Skydel) // TODO: This function becomes obsolete, once the ImuStream is deactivated due to the 'InstinctDataStream'
    {
        return n_calcGravitation_WGS84_Skydel(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::Somigliana)
    {
        return n_calcGravitation_SomiglianaAltitude(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::EGM96)
    {
        return n_calcGravitation_EGM96(lla_position);
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d n_calcGravitation_SomiglianaAltitude(const double& latitude, const double& altitude)
{
    // eq 6.16 has a fault in the denominator, it should be a sin^2(latitude)
    double g_0 = 9.7803253359 * (1.0 + 1.931853e-3 * std::pow(std::sin(latitude), 2))
                 / std::sqrt(1.0 - InsConst::WGS84_e_squared * std::pow(std::sin(latitude), 2));

    // Altitude compensation (Matlab example from Chapter 6_GNSS_INS_1 - glocal.m)
    double k = 1
               - (2 * altitude / InsConst::WGS84_a)
                     * (1 + InsConst::WGS84_f
                        + (std::pow(InsConst::omega_ie * InsConst::WGS84_a, 2))
                              * (InsConst::WGS84_b / InsConst::WGS84_MU))
               + 3 * std::pow(altitude / InsConst::WGS84_a, 2);

    return { 0.0, 0.0, k * g_0 };
}

Eigen::Vector3d n_calcGravitation_WGS84_Skydel(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 spheroid plus local altitude --> possible error!! altitude in lla should be added rather than subtracted!
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    double gravitationMagnitude = InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
                                  - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                  - std::pow(InsConst::omega_ie_Skydel, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

Eigen::Vector3d n_calcGravitation_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Magnitude of the gravity, i.e. without orientation
    double gravitationMagnitude = InsConst::WGS84_MU * std::pow(radiusSpheroid, -2.0)
                                  - 3 * InsConst::WGS84_MU * InsConst::WGS84_J * std::pow(InsConst::WGS84_a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                  - std::pow(InsConst::omega_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

Eigen::Vector3d n_calcGravitation_EGM96(const Eigen::Vector3d& lla_position, int ndegree)
{
    using internal::egm96Coeffs;
    using internal::associatedLegendre;

    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_position);

    // Geocentric latitude determination from Groves (2013) - eq. (2.114)
    double latitudeGeocentric = std::atan(e_position(2) / std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1)));

    // Spherical coordinates
    double radius = std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1) + e_position(2) * e_position(2));
    double elevation = M_PI_2 - latitudeGeocentric; // [rad]
    double azimuth = lla_position(1);               // [rad]

    // Gravitation vector in local-navigation frame coordinates in [m/s^2]
    Eigen::Vector3d n_gravity = Eigen::Vector3d::Zero();

    double Pnm = 0;
    double Pnmd = 0;

    auto coeffsRows = egm96Coeffs.size();

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    auto [P, Pd] = associatedLegendre(ndegree, elevation);

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
            n_gravity(0) += std::pow((InsConst::WGS84_a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnmd;
            n_gravity(1) += std::pow((InsConst::WGS84_a / radius), nd) * md * (C * std::sin(md * azimuth) - S * std::cos(md * azimuth)) * Pnm;
            n_gravity(2) += (nd + 1.0) * std::pow((InsConst::WGS84_a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnm;
        }
    }

    return { -InsConst::WGS84_MU / (radius * radius) * n_gravity(0),
             (1.0 / std::sin(elevation)) * (-InsConst::WGS84_MU / (radius * radius)) * n_gravity(1),
             InsConst::WGS84_MU / (radius * radius) * (1.0 + n_gravity(2)) };
}

Eigen::Vector3d n_calcCentrifugalAcceleration(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84_b, 2.0) / std::pow(InsConst::WGS84_a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    double radiusEarthBody = InsConst::WGS84_a * (1.0 - InsConst::WGS84_f * std::pow(std::sin(latitudeGeocentric), 2.0)) + std::abs(altitude);

    // Centrifugal acceleration - components North and Down (see 'GUT User Guide' eq. 7.4.2)
    double centrifugalN = InsConst::omega_ie * InsConst::omega_ie * radiusEarthBody * std::sin(M_PI_2 - latitudeGeocentric) * std::cos(M_PI_2 - latitudeGeocentric);
    double centrifugalD = InsConst::omega_ie * InsConst::omega_ie * radiusEarthBody * std::sin(M_PI_2 - latitudeGeocentric) * std::sin(M_PI_2 - latitudeGeocentric);

    return { -centrifugalN, 0, -centrifugalD };
}

} // namespace NAV