// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Ellipsoid.hpp
/// @brief Functions concerning the ellipsoid model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-11-28

#pragma once

#include "Navigation/Constants.hpp"

namespace NAV
{

/// @brief r_eS^e The distance of a point on the Earth's surface from the center of the Earth
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] R_E Prime vertical radius of curvature (East/West) in [m]
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Geocentric Radius in [m]
/// @note \cite Groves2013 Groves, ch. 2.4.7, eq. 2.137, p. 71
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Scalar calcGeocentricRadius(const Scalar& latitude, const Scalar& R_E, const Scalar& e_squared = InsConst<Scalar>::WGS84::e_squared)
{
    return R_E * std::sqrt(std::pow(std::cos(latitude), 2) + std::pow((1.0 - e_squared) * std::sin(latitude), 2));
}

/// @brief Calculates the North/South (meridian) earth radius
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return North/South (meridian) earth radius [m]
/// @note See \cite Groves2013 Groves, ch. 2.4.2, eq. 2.105, p. 59
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.83, p. 49
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Scalar calcEarthRadius_N(const Scalar& latitude, const Scalar& a = InsConst<>::WGS84::a, const Scalar& e_squared = InsConst<>::WGS84::e_squared)
{
    Scalar k = std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));

    // North/South (meridian) earth radius [m]
    return a * (1 - e_squared) / std::pow(k, 3);
}

/// @brief Calculates the East/West (prime vertical) earth radius
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return East/West (prime vertical) earth radius [m]
/// @note See \cite Groves2013 Groves, ch. 2.4.2, eq. 2.106, p. 59
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.84, p. 49
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Scalar calcEarthRadius_E(const Scalar& latitude, const Scalar& a = InsConst<Scalar>::WGS84::a, const Scalar& e_squared = InsConst<>::WGS84::e_squared)
{
    // East/West (prime vertical) earth radius [m]
    return a / std::sqrt(1 - e_squared * std::pow(std::sin(latitude), 2));
}

/// @brief Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
/// @param[in] R_N Meridian radius of curvature in [m]
/// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
/// @return T_rn_p A 3x3 matrix
/// @note See \cite Groves2013 Groves, ch. 2.4.3, eq. 2.119, p. 63
template<typename Derived>
[[nodiscard]] Eigen::Matrix3<typename Derived::Scalar> conversionMatrixCartesianCurvilinear(const Eigen::MatrixBase<Derived>& lla_position,
                                                                                            const typename Derived::Scalar& R_N, const typename Derived::Scalar& R_E)
{
    return Eigen::DiagonalMatrix<typename Derived::Scalar, 3>{ 1.0 / (R_N + lla_position(2)),
                                                               1.0 / ((R_E + lla_position(2)) * std::cos(lla_position(0))),
                                                               -1.0 };
}

/// @brief Measure the distance between two points on a sphere
/// @param[in] lat1 Latitude of first point in [rad]
/// @param[in] lon1 Longitude of first point in [rad]
/// @param[in] lat2 Latitude of second point in [rad]
/// @param[in] lon2 Longitude of second point in [rad]
/// @return The distance in [m]
///
/// @note See Haversine Formula (https://www.movable-type.co.uk/scripts/latlong.html)
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Scalar calcGreatCircleDistance(Scalar lat1, Scalar lon1, Scalar lat2, Scalar lon2)
{
    Scalar R = calcGeocentricRadius(lat1, calcEarthRadius_E(lat1));
    Scalar dLat = lat2 - lat1;
    Scalar dLon = lon2 - lon1;
    Scalar a = std::pow(std::sin(dLat / 2.0), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    Scalar c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c; // meters
}

/// @brief Measure the distance between two points over an ellipsoidal-surface
/// @param[in] lat1 Latitude of first point in [rad]
/// @param[in] lon1 Longitude of first point in [rad]
/// @param[in] lat2 Latitude of second point in [rad]
/// @param[in] lon2 Longitude of second point in [rad]
/// @return The distance in [m]
///
/// @note See Lambert's formula for long lines (https://en.wikipedia.org/wiki/Geographical_distance#Lambert's_formula_for_long_lines)
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
[[nodiscard]] Scalar calcGeographicalDistance(Scalar lat1, Scalar lon1, Scalar lat2, Scalar lon2)
{
    if (lat1 == lat2 && lon1 == lon2)
    {
        return 0;
    }
    // First convert the latitudes 𝜙₁,𝜙₂ of the two points to reduced latitudes 𝛽₁,𝛽₂
    Scalar beta1 = std::atan((1 - InsConst<>::WGS84::f) * std::tan(lat1));
    Scalar beta2 = std::atan((1 - InsConst<>::WGS84::f) * std::tan(lat2));

    // Then calculate the central angle 𝜎 in radians between two points 𝛽₁,𝜆₁ and 𝛽₂,𝜆₂ on a sphere using the
    // Great-circle distance method (law of cosines or haversine formula), with longitudes 𝜆₁ and 𝜆₂ being the same on the sphere as on the spheroid.
    Scalar sigma = calcGreatCircleDistance(beta1, lon1, beta2, lon2)
                   / calcGeocentricRadius(lat1, calcEarthRadius_E(lat1));

    Scalar P = (beta1 + beta2) / 2;
    Scalar Q = (beta2 - beta1) / 2;

    Scalar X = (sigma - std::sin(sigma)) * std::pow((std::sin(P) * std::cos(Q)) / std::cos(sigma / 2), 2);
    Scalar Y = (sigma + std::sin(sigma)) * std::pow((std::cos(P) * std::sin(Q)) / std::sin(sigma / 2), 2);

    return InsConst<>::WGS84::a * (sigma - InsConst<>::WGS84::f / 2.0 * (X + Y));
}

} // namespace NAV