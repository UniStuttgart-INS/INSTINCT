// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Gravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"
#include "Navigation/Constants.hpp"
#include "internal/AssociatedLegendre.hpp"
#include "internal/egm96Coeffs.hpp"

namespace NAV
{
/// Available Gravitation Models
enum class GravitationModel : int
{
    None,         ///< Gravity Model turned off
    WGS84,        ///< World Geodetic System 1984
    WGS84_Skydel, ///< World Geodetic System 1984 implemented by the Skydel Simulator // FIXME: Remove after Skydel uses the same as Instinct
    Somigliana,   ///< Somigliana gravity model
    EGM96,        ///< Earth Gravitational Model 1996
    COUNT,        ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] gravitationModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GravitationModel gravitationModel);

/// @brief Shows a ComboBox to select the gravitation model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] gravitationModel Reference to the gravitation model to select
bool ComboGravitationModel(const char* label, GravitationModel& gravitationModel);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Vector3<Scalar> n_calcGravitation_SomiglianaAltitude(const Scalar& latitude, const Scalar& altitude)
{
    // eq 6.16 has a fault in the denominator, it should be a sin^2(latitude)
    auto g_0 = 9.7803253359 * (1.0 + 1.931853e-3 * std::pow(std::sin(latitude), 2))
               / std::sqrt(1.0 - InsConst::WGS84::e_squared * std::pow(std::sin(latitude), 2));

    // Altitude compensation (Matlab example from Chapter 6_GNSS_INS_1 - glocal.m)
    auto k = 1
             - (2 * altitude / InsConst::WGS84::a)
                   * (1 + InsConst::WGS84::f
                      + (std::pow(InsConst::omega_ie * InsConst::WGS84::a, 2))
                            * (InsConst::WGS84::b / InsConst::WGS84::MU))
             + 3 * std::pow(altitude / InsConst::WGS84::a, 2);

    return { 0.0, 0.0, k * g_0 };
}

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential. However, the north component of the centrifugal acceleration is neglected in order to match the implementation of Skydel's 'ImuPlugin'
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Skydel API plug-in 'skydel_plugin/source/library/inertial_math/Sources/source/gravity.cpp'
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Vector3<Scalar> n_calcGravitation_WGS84_Skydel(const Scalar& latitude, const Scalar& altitude)
{
    // geocentric latitude determination from geographic latitude
    auto latitudeGeocentric = std::atan((std::pow(InsConst::WGS84::b, 2.0) / std::pow(InsConst::WGS84::a, 2.0)) * std::tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 spheroid plus local altitude --> possible error!! altitude in lla should be added rather than subtracted!
    auto radiusSpheroid = InsConst::WGS84::a * (1.0 - InsConst::WGS84::f * std::pow(std::sin(latitudeGeocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    auto gravitationMagnitude = InsConst::WGS84::MU * std::pow(radiusSpheroid, -2.0)
                                - 3 * InsConst::WGS84::MU * InsConst::WGS84::J2 * std::pow(InsConst::WGS84::a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                - std::pow(InsConst::omega_ie_Skydel, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential.
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See 'INS-Projects/INSTINCT/SpecificLiterature/GravityPotentialWGS84' in NC folder (eq. (3) derived after 'r')
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Eigen::Vector3<Scalar> n_calcGravitation_WGS84(const Scalar& latitude, const Scalar& altitude)
{
    // Geocentric latitude determination from geographic latitude
    auto latitudeGeocentric = std::atan((std::pow(InsConst::WGS84::b, 2.0) / std::pow(InsConst::WGS84::a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    auto radiusSpheroid = InsConst::WGS84::a * (1.0 - InsConst::WGS84::f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Magnitude of the gravity, i.e. without orientation
    auto gravitationMagnitude = InsConst::WGS84::MU * std::pow(radiusSpheroid, -2.0)
                                - 3 * InsConst::WGS84::MU * InsConst::WGS84::J2 * std::pow(InsConst::WGS84::a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                - std::pow(InsConst::omega_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the EGM96 spherical harmonic model (up to order 10)
/// @param[in] lla_position [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] ndegree Degree of the EGM96 (1 <= ndegree <= 10)
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Groves (2013) Chapter 2.4.3 and 'GUT User Guide' (2018) Chapter 7.4
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> n_calcGravitation_EGM96(const Eigen::MatrixBase<Derived>& lla_position, size_t ndegree = 10)
{
    using internal::egm96Coeffs;
    using internal::associatedLegendre;

    auto e_position = trafo::lla2ecef_WGS84(lla_position);

    // Geocentric latitude determination from Groves (2013) - eq. (2.114)
    auto latitudeGeocentric = std::atan(e_position(2) / std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1)));

    // Spherical coordinates
    auto radius = std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1) + e_position(2) * e_position(2));
    auto elevation = M_PI_2 - latitudeGeocentric; // [rad]
    auto azimuth = lla_position(1);               // [rad]

    // Gravitation vector in local-navigation frame coordinates in [m/s^2]
    Eigen::Vector3<typename Derived::Scalar> n_gravitation = Eigen::Vector3<typename Derived::Scalar>::Zero();

    typename Derived::Scalar Pnm = 0;
    typename Derived::Scalar Pnmd = 0;

    auto coeffsRows = egm96Coeffs.size();

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    auto [P, Pd] = associatedLegendre(static_cast<double>(elevation), ndegree);

    for (size_t i = 0; i < coeffsRows; i++) // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult) // FIXME: Wrong error message about Eigen (error: The left operand of '*' is a garbage value)
    {
        // Retrieving EGM96 coefficients
        auto n = static_cast<int>(egm96Coeffs.at(i).at(0)); // Degree of the Associated Legendre Polynomial
        auto m = static_cast<int>(egm96Coeffs.at(i).at(1)); // Order of the Associated Legendre Polynomial
        auto C = egm96Coeffs.at(i).at(2);
        auto S = egm96Coeffs.at(i).at(3);

        if (static_cast<size_t>(n) == ndegree + 1)
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

            // Gravity vector from differentiation of the gravity potential in spherical coordinates (see 'GUT User Guide' eq. 7.4.2)
            n_gravitation(0) += std::pow((InsConst::WGS84::a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnmd;
            n_gravitation(1) += std::pow((InsConst::WGS84::a / radius), nd) * md * (C * std::sin(md * azimuth) - S * std::cos(md * azimuth)) * Pnm;
            n_gravitation(2) += (nd + 1.0) * std::pow((InsConst::WGS84::a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnm;
        }
    }

    return { -InsConst::WGS84::MU / (radius * radius) * n_gravitation(0),
             (1.0 / std::sin(elevation)) * (-InsConst::WGS84::MU / (radius * radius)) * n_gravitation(1),
             InsConst::WGS84::MU / (radius * radius) * (1.0 + n_gravitation(2)) };
}

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth)
/// @param[in] lla_position [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] gravitationModel Gravitation model to use
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
template<typename Derived>
[[nodiscard]] Eigen::Vector3<typename Derived::Scalar> n_calcGravitation(const Eigen::MatrixBase<Derived>& lla_position,
                                                                         GravitationModel gravitationModel = GravitationModel::EGM96)
{
    const typename Derived::Scalar& latitude = lla_position(0);
    const typename Derived::Scalar& altitude = lla_position(2);

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
    return Eigen::Vector3<typename Derived::Scalar>::Zero();
}

} // namespace NAV
