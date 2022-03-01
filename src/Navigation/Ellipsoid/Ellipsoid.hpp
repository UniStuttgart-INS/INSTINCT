/// @file Ellipsoid.hpp
/// @brief Functions concerning the ellipsoid model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-11-28

#pragma once

#include "Navigation/Constants.hpp"

namespace NAV
{

/// @brief Measure the distance between two points on a sphere
/// @param[in] lat1 Latitude of first point in [rad]
/// @param[in] lon1 Longitude of first point in [rad]
/// @param[in] lat2 Latitude of second point in [rad]
/// @param[in] lon2 Longitude of second point in [rad]
/// @return The distance in [m]
///
/// @note See Haversine Formula (https://www.movable-type.co.uk/scripts/latlong.html)
[[nodiscard]] double calcGreatCircleDistance(double lat1, double lon1, double lat2, double lon2);

/// @brief Measure the distance between two points over an ellipsoidal-surface
/// @param[in] lat1 Latitude of first point in [rad]
/// @param[in] lon1 Longitude of first point in [rad]
/// @param[in] lat2 Latitude of second point in [rad]
/// @param[in] lon2 Longitude of second point in [rad]
/// @return The distance in [m]
///
/// @note See Lambert's formula for long lines (https://en.wikipedia.org/wiki/Geographical_distance#Lambert's_formula_for_long_lines)
[[nodiscard]] double calcGeographicalDistance(double lat1, double lon1, double lat2, double lon2);

/// @brief Calculates the North/South (meridian) earth radius
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return North/South (meridian) earth radius [m]
/// @note See \cite Groves2013 Groves, ch. 2.4.2, eq. 2.105, p. 59
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.83, p. 49
[[nodiscard]] double calcEarthRadius_N(const double& latitude, const double& a = InsConst::WGS84_a, const double& e_squared = InsConst::WGS84_e_squared);

/// @brief Calculates the East/West (prime vertical) earth radius
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return East/West (prime vertical) earth radius [m]
/// @note See \cite Groves2013 Groves, ch. 2.4.2, eq. 2.106, p. 59
/// @note See \cite Titterton2004 Titterton, ch. 3.7.2, eq. 3.84, p. 49
[[nodiscard]] double calcEarthRadius_E(const double& latitude, const double& a = InsConst::WGS84_a, const double& e_squared = InsConst::WGS84_e_squared);

/// @brief r_eS^e The distance of a point on the Earth's surface from the center of the Earth
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @param[in] R_E Prime vertical radius of curvature (East/West) in [m]
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @return Geocentric Radius in [m]
/// @note \cite Groves2013 Groves, ch. 2.4.7, eq. 2.137, p. 71
[[nodiscard]] double calcGeocentricRadius(const double& latitude, const double& R_E, const double& e_squared = InsConst::WGS84_e_squared);

/// @brief Conversion matrix between cartesian and curvilinear perturbations to the position
/// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
/// @param[in] R_N Meridian radius of curvature in [m]
/// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
/// @return T_rn_p A 3x3 matrix
/// @note See \cite Groves2013 Groves, ch. 2.4.3, eq. 2.119, p. 63
[[nodiscard]] Eigen::Matrix3d conversionMatrixCartesianCurvilinear(const Eigen::Vector3d& lla_position, const double& R_N, const double& R_E);

} // namespace NAV