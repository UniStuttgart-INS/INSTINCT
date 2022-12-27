// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GLONASSEphemeris.hpp
/// @brief Galileo Ephemeris information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-12

#pragma once

#include <bitset>

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

namespace NAV
{

/// @brief Broadcasted ephemeris message data
/// @note See \cite GLO-ICD-5.1 GLO-ICD-5.1, ch. 4., p. 25ff
class GLONASSEphemeris final : public SatNavData
{
  public:
    /// @brief Constructor
    /// @param[in] toc Time the Clock information is calculated (Time of Clock)
    /// @param[in] tau_c Coefficient of linear polynomial of time system difference [s]
    /// @param[in] tau_n SV clock bias [s]
    /// @param[in] gamma_n SV relative frequency bias
    /// @param[in] health Health flag
    /// @param[in] pos Position at reference time in PZ90 frame [m]
    /// @param[in] vel Velocity at reference time in PZ90 frame [m/s]
    /// @param[in] accelLuniSolar Accelerations due to lunar-solar gravitational perturbation in PZ90 frame [m/s^2]
    /// @param[in] frequencyNumber Frequency number (-7 ... +13) (-7 ...+6 (ICD 5.1))
    GLONASSEphemeris(const InsTime& toc, double tau_c,
                     double tau_n, double gamma_n, bool health,
                     Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d accelLuniSolar,
                     int8_t frequencyNumber);

#ifdef TESTING
    /// @brief Constructor for pasting raw data from Nav files
    /// @param[in] year Time of Clock year
    /// @param[in] month Time of Clock month
    /// @param[in] day Time of Clock day
    /// @param[in] hour Time of Clock hour
    /// @param[in] minute Time of Clock minute
    /// @param[in] second Time of Clock second
    /// @param[in] tau_n SV clock bias [s]
    /// @param[in] gamma_n SV relative frequency bias
    /// @param[in] messageFrameTime Message frame time (tk+(nd*86400)) in seconds of the UTC week
    /// @param[in] satPos_x Satellite position X (km)
    /// @param[in] satVel_x velocity X dot (km/sec)
    /// @param[in] satAccel_x X acceleration (km/sec2)
    /// @param[in] health Health
    /// @param[in] satPos_y Satellite position Y (km)
    /// @param[in] satVel_y velocity Y dot (km/sec)
    /// @param[in] satAccel_y Y acceleration (km/sec2)
    /// @param[in] frequencyNumber Frequency number (-7 ... +13) (-7 ...+6 (ICD 5.1))
    /// @param[in] satPos_z Satellite position Z (km)
    /// @param[in] satVel_z velocity Z dot (km/sec)
    /// @param[in] satAccel_z Z acceleration (km/sec2)
    /// @param[in] ageOfOperationInfo Age of oper. information (days) (E)
    /// @param[in] statusFlags Status Flags
    /// @param[in] L1L2groupDelayDifference L1/L2 group delay difference Δτ (seconds)
    /// @param[in] URAI URAI; GLO-M/K only – raw accuracy index F_T.
    /// @param[in] healthFlags Health Flags
    GLONASSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second,
                     double tau_n, double gamma_n, double messageFrameTime,
                     double satPos_x, double satVel_x, double satAccel_x, double health,
                     double satPos_y, double satVel_y, double satAccel_y, double frequencyNumber,
                     double satPos_z, double satVel_z, double satAccel_z, double ageOfOperationInfo,
                     double statusFlags = 0.0, double L1L2groupDelayDifference = 0.0, double URAI = 0.0, double healthFlags = 0.0);
#endif

    /// @brief Destructor
    ~GLONASSEphemeris() final = default;
    /// @brief Copy constructor
    GLONASSEphemeris(const GLONASSEphemeris&) = default;
    /// @brief Move constructor
    GLONASSEphemeris(GLONASSEphemeris&&) = default;
    /// @brief Copy assignment operator
    GLONASSEphemeris& operator=(const GLONASSEphemeris&) = delete;
    /// @brief Move assignment operator
    GLONASSEphemeris& operator=(GLONASSEphemeris&&) = delete;

    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    /// @note See \cite GLO-ICD-5.1 GLO-ICD-5.1, Appendix 3, p. 50ff
    [[nodiscard]] Pos calcSatellitePos(const InsTime& transTime) const final;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    /// @note See \cite GLO-ICD-5.1 GLO-ICD-5.1, Appendix 3, p. 50ff
    [[nodiscard]] PosVel calcSatellitePosVel(const InsTime& transTime) const final;
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time to calculate the satellite position for
    /// @note See \cite GLO-ICD-5.1 GLO-ICD-5.1, Appendix 3, p. 50ff
    [[nodiscard]] PosVelAccel calcSatellitePosVelAccel(const InsTime& transTime) const final;

    /// @brief Calculates the Variance of the satellite position in [m]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

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
    /// @param[out] clkBias Satellite Clock bias [s]
    /// @param[out] clkDrift Satellite Clock drift [s/s]
    /// @param[out] e_pos The Earth fixed coordinates in WGS84 frame of the satellite at the requested time [m] (nullptr disables calculation)
    /// @param[out] e_vel The WGS84 frame velocity of the satellite at the requested time [m/s] (nullptr disables calculation)
    /// @param[out] e_accel The WGS84 frame acceleration of the satellite at the requested time [m/s^2] (nullptr disables calculation)
    /// @note See \cite GLO-ICD-5.1 GLO ICD A.3.1.1 p.50ff
    void calcSatellitePosVelAccelClk(const InsTime& recvTime, double dist, double* clkBias, double* clkDrift,
                                     Eigen::Vector3d* e_pos = nullptr, Eigen::Vector3d* e_vel = nullptr, Eigen::Vector3d* e_accel = nullptr) const;

    // #######################################################################################################
    //                                                Members
    // #######################################################################################################

    /// Coefficient of linear polynomial of time system difference [s]
    const double tau_c;

    /// Toe Time of clock [s] (Reference time, ephemeris parameters)
    const InsTime toc;
    /// SV clock bias [s]
    const double tau_n;
    /// SV relative frequency bias
    const double gamma_n;
    /// Health
    const bool health;
    /// Position at reference time in PZ90 frame [m]
    const Eigen::Vector3d pos;
    /// Velocity at reference time in PZ90 frame [m/s]
    const Eigen::Vector3d vel;
    /// Accelerations due to lunar-solar gravitational perturbation in PZ90 frame [m/s^2]
    const Eigen::Vector3d accelLuniSolar;
    /// Frequency number (-7 ... +13) (-7 ...+6 (ICD 5.1))
    const int8_t frequencyNumber;

  private:
    /// Integration step size in [s]
    static constexpr double _h = 60.0;
};

} // namespace NAV