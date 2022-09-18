// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Ephemeris.hpp
/// @brief Broadcast Ephemeris data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-07

#pragma once

#include <Eigen/Core>

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// Satellite Clock corrections
struct SatelliteClockCorrections
{
    InsTime transmitTime{}; ///< Transmit time of the signal
    double bias{};          ///< Satellite clock bias [s]
    double drift{};         ///< Satellite clock drift [s/s]
};

/// @brief Broadcasted ephemeris message data
/// @note See \cite IS-GPS-200M IS-GPS-200M p 105ff
///
/// \image html GPS-satellite-orbits.png "Description of the GPS satellite orbits"
struct GPSEphemeris
{
    // ------------------------------------------ Time Parameters --------------------------------------------

    /// Toe Time of Ephemeris [s] (Reference time, ephemeris parameters)
    InsTime toe;
    /// Toc Time of Clock [s] (Reference time, clock parameters)
    InsTime toc;
    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    std::array<double, 3> a{};

    // --------------------------------------- Keplerian Parameters ------------------------------------------

    double sqrt_A{};  ///< Square root of the semi-major axis [m^1/2]
    double e{};       ///< Eccentricity [-]
    double i_0{};     ///< Inclination angle at reference time [rad]
    double Omega_0{}; ///< Longitude of the ascending node at reference time [rad]
    double omega{};   ///< Argument of perigee [rad]
    double M_0{};     ///< Mean anomaly at reference time [rad]

    // -------------------------------------- Pertubation Parameters -----------------------------------------

    double delta_n{};   ///< Mean motion difference from computed value [rad/s]
    double Omega_dot{}; ///< Rate of change of right ascension [rad/s]
    double i_dot{};     ///< Rate of change of inclination [rad/s]
    double Cus{};       ///< Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    double Cuc{};       ///< Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    double Cis{};       ///< Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    double Cic{};       ///< Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    double Crs{};       ///< Amplitude of the sine harmonic correction term to the orbit radius [m]
    double Crc{};       ///< Amplitude of the cosine harmonic correction term to the orbit radius [m]

    // ------------------------------------------ Signal accuracy --------------------------------------------

    /// Signal accuracy [m]
    ///
    /// - GPS: SV accuracy (see \cite IS-GPS-200M GPS ICD ch. 20.3.3.3.1.3, p.92ff)
    /// - GAL: SISA (Signal in space accuracy) (see \cite GAL-ICD-2.0 GAL ICD ch. 5.1.12, p.58)
    double signalAccuracy{};

    /// Signal Health
    ///
    /// - GPS: SV health (bits 17-22 w 3 sf 1) see \cite IS-GPS-200M GPS ICD ch. 30.3.3.1.1.2, p. 160
    /// - GAL: SV health (FLOAT converted to INTEGER) see \cite GAL-ICD-2.0 Galileo ICD ch. 5.1.9.3, p.52
    ///            Bit 0: E1B DVS       Bits 1-2: E1B HS          Bit 3: E5a DVS
    ///            Bits 4-5: E5a HS     Bit 6: E5b DVS            Bits 7-8: E5b HS
    uint16_t svHealth{};

    // ---------------------------------------- L1 or L2 Correction ------------------------------------------

    double T_GD{};       ///< L1 and L2 correction term [s]
    double BGD_E1_E5a{}; ///< GAL - Broadcast Group Delay [s]
    double BGD_E1_E5b{}; ///< GAL - Broadcast Group Delay [s]

    // #######################################################################################################

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] SatelliteClockCorrections calcSatelliteClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const;

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    /// @param[in] satSys Satellite System
    /// @param[out] e_pos The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m] (nullptr disables calculation)
    /// @param[out] e_vel The WGS84 frame velocity of the satellite at the requested time [m/s] (nullptr disables calculation)
    /// @param[out] e_accel The WGS84 frame acceleration of the satellite at the requested time [m/s^2] (nullptr disables calculation)
    /// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    void calcSatellitePosVelAccel(const InsTime& transTime, const SatelliteSystem& satSys,
                                  Eigen::Vector3d* e_pos = nullptr, Eigen::Vector3d* e_vel = nullptr, Eigen::Vector3d* e_accel = nullptr) const;

    /// @brief Converts a GALILEO SISA (Signal in space accuracy) value to it's index
    /// @param[in] val SISA value in [m]
    /// @return The SISA index
    ///
    /// @note See \cite GAL-ICD-2.0 GAL ICD ch. 5.1.12, p.58
    static uint8_t galSisaVal2Idx(double val);

    /// @brief Converts a GALILEO SISA (Signal in space accuracy) index to it's value
    /// @param[in] idx The SISA index
    /// @return SISA value in [m]
    ///
    /// @note See \cite GAL-ICD-2.0 GAL ICD ch. 5.1.12, p.58
    static double galSisaIdx2Val(uint8_t idx);

    /// @brief Converts a GPS URA (user range accuracy) value to it's index
    /// @param[in] val URA value in [m]
    /// @return The URA index
    ///
    /// @note See \cite IS-GPS-200M GPS ICD ch. 20.3.3.3.1.3, p.92ff
    static uint8_t gpsUraVal2Idx(double val);

    /// @brief Converts a GPS URA (user range accuracy) index to it's value
    /// @param[in] idx The URA index
    /// @return URA value in [m]
    ///
    /// @note See \cite IS-GPS-200M GPS ICD ch. 20.3.3.3.1.3, p.92ff
    static double gpsUraIdx2Val(uint8_t idx);
};

/// @brief Broadcasted ephemeris message data for GLONASS satellites
struct GLONASSEphemeris
{
    /// Coefficient of linear polynomial of time system difference [s]
    double tau_c{};

    /// Toe Time of clock [s] (Reference time, ephemeris parameters)
    InsTime toc;
    /// SV clock bias [s]
    double tau_n{};
    /// SV relative frequency bias
    double gamma_n{};
    /// Position at reference time in PZ90 frame [m]
    Eigen::Vector3d pos;
    /// Velocity at reference time in PZ90 frame [m/s]
    Eigen::Vector3d vel;
    /// Accelerations due to lunar-solar gravitational perturbation in PZ90 frame [m/s^2]
    Eigen::Vector3d accelLuniSolar;
    /// Frequency number (-7 ... +13) (-7 ...+6 (ICD 5.1))
    int8_t frequencyNumber = -128;

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    ///
    /// Re-calculation of ephemeris from instant t_e to instant t_i within the interval of measurement
    ///     ( |t_i - t_e| < 15 minutes ) is performed using technique of numerical integration.
    /// The equations are integrated in direct absolute geocentric coordinate system, connected with
    ///     current equator and vernal equinox, using 4th order Runge-Kutta technique
    /// The ICD specifies the following algorithm:
    ///     1. Coordinate transformation PZ90 to absolute geocentric coordinate system (skipped)
    ///     2. Numerical integration with 4th order Runge-Kutta technique
    ///     3. Coordinates transformation back to the PZ-90 reference system (skipped)
    ///     4. Coordinates transformation PZ-90 to ECEF WGS84 reference system
    /// As the transformation of the PZ90 to absolute system depends on the true sidereal time which can't be calculated easily,
    /// this step is skipped without loosing too much accuracy.
    ///
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] satSys Satellite System (GLONASS or SBAS)
    /// @param[out] clkBias Satellite Clock bias [s]
    /// @param[out] clkDrift Satellite Clock drift [s/s]
    /// @param[out] e_pos The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m] (nullptr disables calculation)
    /// @param[out] e_vel The WGS84 frame velocity of the satellite at the requested time [m/s] (nullptr disables calculation)
    /// @param[out] e_accel The WGS84 frame acceleration of the satellite at the requested time [m/s^2] (nullptr disables calculation)
    /// @note See \cite GLO-ICD-5.1 GLO ICD A.3.1.1 p.50ff
    void calcSatellitePosVelAccelClk(const InsTime& recvTime, double dist, const SatelliteSystem& satSys, double* clkBias, double* clkDrift,
                                     Eigen::Vector3d* e_pos = nullptr, Eigen::Vector3d* e_vel = nullptr, Eigen::Vector3d* e_accel = nullptr) const;

  private:
    /// Integration step size in [s]
    static constexpr double _h = 60.0;
};

} // namespace NAV
