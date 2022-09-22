// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Constants.hpp
/// @brief Holds all Constants
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-21

#pragma once

#include <cmath>
#include "util/Eigen.hpp"
#include <gcem.hpp>

namespace NAV::InsConst
{
/// @brief Conversion factor between latitude and longitude in [rad] to [pseudometre]
constexpr double pseudometre = 6370000;

/// Speed of light [m/s]
constexpr double C = 299792458.0;

/// Standard gravity in [m / s^2]
constexpr double G_NORM = 9.80665;

/// @brief World Geodetic System 1984
namespace WGS84
{
/// Semi-major axis = equatorial radius
constexpr double a = 6378137.0;
/// Flattening f = (a-b)/a
constexpr double f = 1.0 / 298.257223563;
/// Semi-minor axis = polar radius
constexpr double b = a - f * a;
/// Square of the first eccentricity of the ellipsoid
constexpr double e_squared = 2 * f - f * f;
/// Gravitational constant (mass of Earth’s atmosphere included) [m³/s²]
constexpr double MU = 3.986004418e14;
/// Dynamic form factor, derived [-]
constexpr double J2 = 1.081874e-3;
/// Earth rotation rate in [rads/s]
constexpr double omega_ie = 7.2921151467e-05;

} // namespace WGS84

/// @brief Geodetic Reference System 1980
namespace GRS80
{
/// Semi-major axis = equatorial radius
constexpr double a = 6378137;
/// Flattening f = (a-b)/a
constexpr double f = 1.0 / 298.257222101;
/// Semi-minor axis = polar radius
constexpr double b = a - f * a;
/// Square of the first eccentricity of the ellipsoid
constexpr double e_squared = 2 * f - f * f;

} // namespace GRS80

/// @brief Parametry Zemli 1990 goda (see \cite PZ-90.11)
namespace PZ90
{
inline namespace PZ90_11
{
/// Semi-major axis = equatorial radius
constexpr double a = 6378136;
/// Flattening f = (a-b)/a
constexpr double f = 1.0 / 298.25784;
/// Semi-minor axis = polar radius
constexpr double b = a - f * a;
/// Square of the first eccentricity of the ellipsoid
constexpr double e_squared = 2 * f - f * f;
/// Gravitational constant (mass of Earth’s atmosphere included) [m³/s²]
constexpr double MU = 3.986004418e14;
/// Earth rotation rate in [rads/s]
constexpr double omega_ie = 7.292115e-05;

/// Second degree zonal coefficient of normal potential [-]
constexpr double J2 = 1.08262575e-3;

} // namespace PZ90_11
} // namespace PZ90

/// @brief GPS related constants
namespace GPS
{
/// Gravitational constant GPS [m³/s²]. See \cite IS-GPS-200M IS-GPS-200M p. 106
constexpr double MU = 3.986005e+14;
/// Earth angular velocity GPS [rad/s]. See \cite IS-GPS-200M IS-GPS-200M p. 106
constexpr double omega_ie = WGS84::omega_ie;
/// Relativistic constant F for GPS clock corrections [s/√m] (-2*√µ/c²)
constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
/// Earth Equatorial Radius [m]
constexpr double R_E = WGS84::a;
/// Oblate Earth Gravity Coefficient [-]
constexpr double J2 = 1.0826262e-3;

} // namespace GPS

/// @brief GLONASS related constants (see \cite GLO-ICD-5.1 GLONASS ICD 5.1 Table 3.2)
namespace GLO
{
/// Semi-major axis = equatorial radius
constexpr double a = PZ90::a;
/// Gravitational constant GLONASS [m³/s²]
constexpr double MU = PZ90::MU;
/// Earth angular velocity GLONASS [rad/s]
constexpr double omega_ie = PZ90::omega_ie;
/// Second degree zonal coefficient of normal potential [-]
constexpr double J2 = PZ90::J2;
/// Normalized harmonic of the normal geopotential [-]
constexpr double C20_bar = 1.0 / gcem::sqrt(5.0) * J2;
/// Second zonal coefficient of spherical harmonic expansion [-]
constexpr double C20 = -J2;

} // namespace GLO

/// @brief GALILEO related constants
namespace GAL
{
/// Earth angular velocity GALILEO [rad/s]
constexpr double omega_ie = WGS84::omega_ie;
/// Earth gravitational constant GALILEO [m³/s²]
constexpr double MU = 3.986004418e+14;
/// Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);

} // namespace GAL

/// @brief Nominal mean angular velocity of the Earth in [rad/s]
/// @note D. D. McCarthy, G. Petit (Hrsg.): IERS Conventions (2003) (IERS Technical Note No. 32), Kap. 1: General Definitions and Numerical Standards.
///         ftp://tai.bipm.org/iers/conv2003/chapter1/tn32_c1.pdf
constexpr double omega_ie = WGS84::omega_ie;
/// @brief Nominal mean angular velocity of the Earth in [rad/s]. Value implemented by the Skydel GNSS simulator (for compatibility with Skydel's IMU plugin)
constexpr double omega_ie_Skydel = 7.2921155e-5; // FIXME: Skydel (for compatibility with Skydel's IMU plugin)

/// ω_ie_e = ω_ie_i Nominal mean angular velocity of the Earth in [rad/s], in earth coordinates
const static Eigen::Vector3d e_omega_ie{ 0.0, 0.0, omega_ie };

} // namespace NAV::InsConst