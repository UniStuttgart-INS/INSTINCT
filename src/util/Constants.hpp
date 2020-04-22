/**
 * @file Constants.hpp
 * @brief Holds all Constants
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-21
 */

#pragma once

namespace NAV::InsConst
{
const double C = 299792458.0; ///< speed of light (m/s)
const double C2 = C * C;      ///< speed of light, squared (m^2/s^2)
const double C3 = C2 * C;     ///< speed of light, third order (m^3/s^3)

const double RHO = 180.0 / M_PI;

const double WGS84_a = 6378137.0;
const double WGS84_f = 1.0 / 298.257223563;
const double GRS80_a = 6378137;
const double GRS80_f = 1.0 / 298.257222101;

const double GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F = -4.442807633e-10;
const double GPS_RATIO_OF_SQUARED_FREQUENCIES_L1_OVER_L2 = 1.6469444444444444444444444444444;
const double GPS_UNIVERSAL_GRAVITY_CONSTANT = 3.986005e+14; ///< gravitational constant GPS
const double GPS_OMG = 7.2921151467e-05;                    ///< earth angular velocity (rad/s) GPS, WGS84 earth rotation rate

const double GLO_OMG = 7.292115e-5; ///< earth angular velocity (rad/s) GLONASS
const double GLO_MU = 3.986005e+14; ///< gravitational constant GLONASS
const double GLO_J2 = 1.0826257e-3; ///< 2nd zonal harmonic of geopot GLONASS
const double GLO_RE = 6378136.0;    ///< radius GLONASS

const double GAL_OMG = 7.2921151467e-5; ///< earth angular velocity (rad/s) GALILEO
const double GAL_MU = 3.986004418e+14;  ///< earth gravitational constant GALILEO

} // namespace NAV::InsConst