/// @file Constants.hpp
/// @brief Holds all Constants
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-21

#pragma once

#include <cmath>
#include "util/Eigen.hpp"

namespace NAV::InsConst
{
/// @brief Nominal mean angular velocity of the Earth in [rad/s]
/// @note D. D. McCarthy, G. Petit (Hrsg.): IERS Conventions (2003) (IERS Technical Note No. 32), Kap. 1: General Definitions and Numerical Standards.
///         ftp://tai.bipm.org/iers/conv2003/chapter1/tn32_c1.pdf
constexpr double angularVelocity_ie = 7.2921151467e-05;

/// ω_ie_e = ω_ie_i Nominal mean angular velocity of the Earth in [rad/s], in earth coordinates
const static Eigen::Vector3d angularVelocity_ie_e{ 0.0, 0.0, angularVelocity_ie };

/// Ω_ie_e Cross product Matrix of nominal mean angular velocity of the Earth in [rad/s], in earth coordinates
const static Eigen::Matrix3d angularVelocityCrossProduct_ie_e{ (Eigen::Matrix3d() << 0, -angularVelocity_ie_e(2), angularVelocity_ie_e(1),
                                                                angularVelocity_ie_e(2), 0, -angularVelocity_ie_e(0),
                                                                -angularVelocity_ie_e(1), angularVelocity_ie_e(0), 0)
                                                                   .finished() };

/// Semi-major axis = equatorial radius (World Geodetic System 1984)
constexpr double WGS84_a = 6378137.0;
/// Flattening f = (a-b)/a (World Geodetic System 1984)
constexpr double WGS84_f = 1.0 / 298.257223563;
/// Semi-minor axis = polar radius (World Geodetic System 1984)
constexpr double WGS84_b = WGS84_a - WGS84_f * WGS84_a;
/// Square of the first eccentricity of the ellipsoid (World Geodetic System 1984)
constexpr double WGS84_e_squared = 2 * WGS84_f - WGS84_f * WGS84_f;
/// Gravitational constant (mass of Earth’s atmosphere included) [m³/s²] (World Geodetic System 1984)
constexpr double WGS84_MU = 3.986004418e14;

/// Semi-major axis = equatorial radius (Geodetic Reference System 1980)
constexpr double GRS80_a = 6378137;
/// Flattening f = (a-b)/a (Geodetic Reference System 1980)
constexpr double GRS80_f = 1.0 / 298.257222101;
/// Semi-minor axis = polar radius (Geodetic Reference System 1980)
constexpr double GRS80_b = GRS80_a - GRS80_f * GRS80_a;
/// Square of the first eccentricity of the ellipsoid (Geodetic Reference System 1980)
constexpr double GRS80_e_squared = 2 * GRS80_f - GRS80_f * GRS80_f;

constexpr double C = 299792458.0; ///< speed of light (m/s)
constexpr double C2 = C * C;      ///< speed of light, squared (m^2/s^2)
constexpr double C3 = C2 * C;     ///< speed of light, third order (m^3/s^3)

constexpr double GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F = -4.442807633e-10;
constexpr double GPS_RATIO_OF_SQUARED_FREQUENCIES_L1_OVER_L2 = 1.6469444444444444444444444444444;
constexpr double GPS_UNIVERSAL_GRAVITY_CONSTANT = 3.986005e+14; ///< gravitational constant GPS
constexpr double GPS_OMG = 7.2921151467e-05;                    ///< earth angular velocity (rad/s) GPS, WGS84 earth rotation rate

constexpr double GLO_OMG = 7.292115e-5; ///< earth angular velocity (rad/s) GLONASS
constexpr double GLO_MU = 3.986005e+14; ///< gravitational constant GLONASS
constexpr double GLO_J2 = 1.0826257e-3; ///< 2nd zonal harmonic of geopot GLONASS
constexpr double GLO_RE = 6378136.0;    ///< radius GLONASS

constexpr double GAL_OMG = angularVelocity_ie; ///< earth angular velocity (rad/s) GALILEO
constexpr double GAL_MU = 3.986004418e+14;     ///< earth gravitational constant GALILEO

/// Gravity norm constant [m/s^2]
constexpr double G_NORM = 9.80665;

} // namespace NAV::InsConst