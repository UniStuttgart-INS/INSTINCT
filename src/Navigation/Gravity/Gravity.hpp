/// @file Gravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

#include <Eigen/Dense>

namespace NAV
{

/// Available Gravity Models
enum class GravityModel : int
{
    WGS84,        ///< World Geodetic System 1984
    WGS84_Skydel, ///< World Geodetic System 1984 implemented by the Skydel Simulator // FIXME: Remove after Skydel uses the same as Instinct
    Somigliana,   ///< Somigliana gravity model
    EGM96,        ///< Earth Gravitational Model 1996
    OFF,          ///< Gravity Model turned off
    COUNT,        ///< Amount of items in the enum
};

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth)
/// @param[in] latLonAlt [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] gravityModel Gravitation model to use
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
[[nodiscard]] Eigen::Vector3d calcGravitation_n(const Eigen::Vector3d& latLonAlt, GravityModel gravityModel = GravityModel::EGM96);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] Eigen::Vector3d calcGravitation_n_SomiglianaAltitude(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential, but neglecting the north component of the centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Skydel API plug-in 'skydel_plugin/source/library/inertial_math/Sources/source/gravity.cpp'
[[nodiscard]] Eigen::Vector3d calcGravitation_n_WGS84_Skydel(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using gravity as derived from the gravity potential with correctly oriented centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [m]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See 'INS-Projects/INSTINCT/SpecificLiterature/GravityPotentialWGS84' in NC folder (eq. (3) derived after 'r')
[[nodiscard]] Eigen::Vector3d calcGravitation_n_WGS84(const double& latitude, const double& altitude);

/// @brief Calculates the gravitation (acceleration due to mass attraction of the Earth) at the WGS84 reference ellipsoid
///        using the EGM96 spherical harmonic model (up to order 10) including the centrifugal acceleration
/// @param[in] latLonAlt [ϕ, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
/// @param[in] ndegree Degree of the EGM96 (1 <= ndegree <= 10)
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Groves (2013) Chapter 2.4.3 and 'GUT User Guide' (2018) Chapter 7.4
[[nodiscard]] Eigen::Vector3d calcGravitation_n_EGM96(const Eigen::Vector3d& latLonAlt, int ndegree = 10);

/// @brief Calculates the centrifugal acceleration
/// @param[in] latitude Latitude in [rad]
/// @param[in] altitude Altitude in [rad]
/// @return Gravitation vector in local-navigation frame coordinates in [m/s^2]
///
/// @note See Groves (2013) Chapter 2.4.7, p. 67ff
[[nodiscard]] Eigen::Vector3d calcCentrifugalAcceleration(const double& latitude, const double& altitude);

/// @brief Converts the enum to a string
/// @param[in] gravityModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GravityModel gravityModel);

} // namespace NAV
