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

namespace NAV
{

/// @brief Constants
class InsConst
{
  public:
    /// @brief Default Constructor
    InsConst() = delete;

    /// @brief Conversion factor between latitude and longitude in [rad] to [pseudometre]
    static constexpr double pseudometre = 6370000;

    /// Speed of light [m/s]
    static constexpr double C = 299792458.0;

    /// Standard gravity in [m / s^2]
    static constexpr double G_NORM = 9.80665;

    /// @brief World Geodetic System 1984
    class WGS84
    {
      public:
        /// @brief Default Constructor
        WGS84() = delete;

        /// Semi-major axis = equatorial radius
        static constexpr double a = 6378137.0;
        /// Flattening f = (a-b)/a
        static constexpr double f = 1.0 / 298.257223563;
        /// Semi-minor axis = polar radius
        static constexpr double b = a - f * a;
        /// Square of the first eccentricity of the ellipsoid
        static constexpr double e_squared = 2 * f - f * f;
        /// Gravitational constant (mass of Earth’s atmosphere included) [m³/s²]
        static constexpr double MU = 3.986004418e14;
        /// Dynamic form factor, derived [-]
        static constexpr double J2 = 1.081874e-3;
        /// Earth rotation rate in [rads/s]
        static constexpr double omega_ie = 7.2921151467e-05;
    };

    /// @brief Geodetic Reference System 1980
    class GRS80
    {
      public:
        /// @brief Default Constructor
        GRS80() = delete;

        /// Semi-major axis = equatorial radius
        static constexpr double a = 6378137;
        /// Flattening f = (a-b)/a
        static constexpr double f = 1.0 / 298.257222101;
        /// Semi-minor axis = polar radius
        static constexpr double b = a - f * a;
        /// Square of the first eccentricity of the ellipsoid
        static constexpr double e_squared = 2 * f - f * f;
    };

    /// @brief Parametry Zemli 1990 goda (see \cite PZ-90.11)
    class PZ90
    {
      public:
        /// @brief Default Constructor
        PZ90() = delete;

        /// Semi-major axis = equatorial radius
        static constexpr double a = 6378136;
        /// Flattening f = (a-b)/a
        static constexpr double f = 1.0 / 298.25784;
        /// Semi-minor axis = polar radius
        static constexpr double b = a - f * a;
        /// Square of the first eccentricity of the ellipsoid
        static constexpr double e_squared = 2 * f - f * f;
        /// Gravitational constant (mass of Earth’s atmosphere included) [m³/s²]
        static constexpr double MU = 3.986004418e14;
        /// Earth rotation rate in [rads/s]
        static constexpr double omega_ie = 7.292115e-05;

        /// Second degree zonal coefficient of normal potential [-]
        static constexpr double J2 = 1.08262575e-3;
    };

    /// @brief GPS related constants
    class GPS
    {
      public:
        /// @brief Default Constructor
        GPS() = delete;

        /// Gravitational constant GPS [m³/s²]. See \cite IS-GPS-200M IS-GPS-200M p. 106
        static constexpr double MU = 3.986005e+14;
        /// Earth angular velocity GPS [rad/s]. See \cite IS-GPS-200M IS-GPS-200M p. 106
        static constexpr double omega_ie = WGS84::omega_ie;
        /// Relativistic constant F for GPS clock corrections [s/√m] (-2*√µ/c²)
        static constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
        /// Earth Equatorial Radius [m]
        static constexpr double R_E = WGS84::a;
        /// Oblate Earth Gravity Coefficient [-]
        static constexpr double J2 = 1.0826262e-3;
    };

    /// @brief GLONASS related constants (see \cite GLO-ICD-5.1 GLONASS ICD 5.1 Table 3.2)
    class GLO
    {
      public:
        /// @brief Default Constructor
        GLO() = delete;

        /// Semi-major axis = equatorial radius
        static constexpr double a = PZ90::a;
        /// Gravitational constant GLONASS [m³/s²]
        static constexpr double MU = PZ90::MU;
        /// Earth angular velocity GLONASS [rad/s]
        static constexpr double omega_ie = 7.2921151467e-05;
        /// Second degree zonal coefficient of normal potential [-]
        static constexpr double J2 = PZ90::J2;
        /// Normalized harmonic of the normal geopotential [-]
        static constexpr double C20_bar = 1.0 / gcem::sqrt(5.0) * J2;
        /// Second zonal coefficient of spherical harmonic expansion [-]
        static constexpr double C20 = -J2;
    };

    /// @brief GALILEO related constants
    class GAL
    {
      public:
        /// @brief Default Constructor
        GAL() = delete;

        /// Earth angular velocity GALILEO [rad/s]
        static constexpr double omega_ie = WGS84::omega_ie;
        /// Earth gravitational constant GALILEO [m³/s²]
        static constexpr double MU = 3.986004418e+14;
        /// Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
        static constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
    };

    /// @brief BeiDou related constants (China Geodetic Coordinate System 2000 (CGCS2000))
    class BDS
    {
      public:
        /// @brief Default Constructor
        BDS() = delete;

        /// Semi-major axis = equatorial radius
        static constexpr double a = 6378137.0;
        /// Flattening f = (a-b)/a
        static constexpr double f = 1.0 / 298.257222101;
        /// Semi-minor axis = polar radius
        static constexpr double b = a - f * a;
        /// Square of the first eccentricity of the ellipsoid
        static constexpr double e_squared = 2 * f - f * f;
        /// Earth angular velocity BeiDou (CGCS2000) [rad/s]
        static constexpr double omega_ie = 7.2921150e-5;
        /// Earth gravitational constant BeiDou (CGCS2000) [m³/s²]
        static constexpr double MU = 3.986004418e+14;
        /// Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
        static constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
    };

    /// @brief QZSS related constants
    class QZSS
    {
      public:
        /// @brief Default Constructor
        QZSS() = delete;

        /// Earth angular velocity QZSS [rad/s]
        static constexpr double omega_ie = WGS84::omega_ie;
        /// Earth gravitational constant QZSS [m³/s²]
        static constexpr double MU = 3.986005e+14;
        /// Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
        static constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
    };

    /// @brief IRNSS related constants
    class IRNSS
    {
      public:
        /// @brief Default Constructor
        IRNSS() = delete;

        /// Earth angular velocity IRNSS [rad/s]
        static constexpr double omega_ie = WGS84::omega_ie;
        /// Earth gravitational constant IRNSS [m³/s²]
        static constexpr double MU = 3.986005e+14;
        /// Relativistic constant F for clock corrections [s/√m] (-2*√µ/c²)
        static constexpr double F = -2.0 * gcem::sqrt(MU) / (C * C);
    };

    /// @brief Nominal mean angular velocity of the Earth in [rad/s]
    /// @note D. D. McCarthy, G. Petit (Hrsg.): IERS Conventions (2003) (IERS Technical Note No. 32), Kap. 1: General Definitions and Numerical Standards.
    ///         ftp://tai.bipm.org/iers/conv2003/chapter1/tn32_c1.pdf
    static constexpr double omega_ie = WGS84::omega_ie;
    /// @brief Nominal mean angular velocity of the Earth in [rad/s]. Value implemented by the Skydel GNSS simulator (for compatibility with Skydel's IMU plugin)
    static constexpr double omega_ie_Skydel = 7.2921155e-5; // FIXME: Skydel (for compatibility with Skydel's IMU plugin)

    /// ω_ie_e = ω_ie_i Nominal mean angular velocity of the Earth in [rad/s], in earth coordinates
    const static inline Eigen::Vector3<double> e_omega_ie{ 0.0, 0.0, omega_ie };

    /// Avogadro’s number. Number of units in one mole of any substance [1/mol].
    /// Units may be electrons, atoms, ions, or molecules, depending on substance or reaction
    static constexpr double N_A = 6.02214076e23;
    /// Boltzmann constant [J/K]
    static constexpr double k_B = 1.380649e-23;
    /// Universal gas constant in [J/K/mol]
    static constexpr double Rg = N_A * k_B;

    /// Molar mass of dry air in [kg/mol]
    static constexpr double dMtr = 28.965e-3;
};

} // namespace NAV