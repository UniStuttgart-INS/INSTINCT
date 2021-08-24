/// @file InsGravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

#include <Eigen/Dense>

namespace NAV::gravity
{
/// @brief Calculates the magnitude of the of local gravity at the WGS84 reference elliposid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Magnitude of the gravity vector in [m/s^2]
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] double gravityMagnitude_SomiglianaAltitude(const double& latitude, const double& altitude);

/// @brief Calculates the local gravity vector at the WGS84 reference ellipsoid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] Eigen::Vector3d gravity_SomiglianaAltitude(const double& latitude, const double& altitude);

/// @brief Calculates the local gravity vector at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential, but neglecting the north component
///        of the centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See Skydel API plug-in 'skydel_plugin/source/library/inertial_math/Sources/source/gravity.cpp'
[[nodiscard]] double gravityMagnitude_WGS84_Skydel(const double& latitude, const double& altitude);

/// @brief Calculates the local gravity vector of a body above the WGS84 reference ellipsoid using gravity
///        as derived from the gravity potential with correctly oriented centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See 'INS-Projects/INSTINCT/SpecificLiterature/GravityPotentialWGS84' in NC folder (eq. (3) derived after 'r')
[[nodiscard]] Eigen::Vector3d gravity_WGS84(const double& latitude, const double& altitude);

/// @brief Calculates a gravity vector that contains the centrifugal acceleration for the Somigliana model (which already has an altitude compensation)
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @param[in] gravityMagnitude Magnitude of gravity on a body according to Somigliana model
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See Groves (2013) Chapter 2.4.7
[[nodiscard]] Eigen::Vector3d centrifugalAcceleration_Somigliana(const double& latitude, const double& altitude, double gravityMagnitude);

/// @brief Calculates a gravity vector that contains the centrifugal acceleration and an altitude compensation for the WGS84 model
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @param[in] gravityMagnitude Magnitude of gravity on a body according to Somigliana model
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See Groves (2013) Chapter 2.4.7
[[nodiscard]] Eigen::Vector3d centrifugalAcceleration_WGS84(const double& latitude, const double& altitude, double gravityMagnitude);

/// @brief Calculates the local gravity vector at the WGS84 reference ellipsoid using the EGM96 spherical harmonic
///        model (up to order 10) including the centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] longitude Longitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @param[in] ndegree Degree of the EGM96 (1 <= ndegree <= 10)
/// @return Gravity vector in [m/s^2] in NED frame
///
/// @note See Groves (2013) Chapter 2.4.3 and 'GUT User Guide' (2018) Chapter 7.4
[[nodiscard]] Eigen::Vector3d gravity_EGM96(const double& latitude, const double& longitude, const double& altitude, int ndegree);

} // namespace NAV::gravity
