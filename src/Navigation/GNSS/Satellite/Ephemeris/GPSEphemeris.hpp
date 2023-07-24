// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GPSEphemeris.hpp
/// @brief GPS Ephemeris information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-02

#pragma once

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// @brief Broadcasted ephemeris message data
/// @note See \cite IS-GPS-200M IS-GPS-200M, Table 20-III, p. 105
///
/// \image html GPS-satellite-orbits.png "Description of the GPS satellite orbits"
class GPSEphemeris final : public SatNavData
{
  public:
    // #######################################################################################################
    //                                                Members
    // #######################################################################################################

    // ------------------------------------------ Time Parameters --------------------------------------------

    /// @brief Time of Clock
    const InsTime toc;

    /// @brief Time of Ephemeris
    const InsTime toe;

    /// @brief Issue of Data, Ephemeris
    ///
    /// Provides the user with a convenient means for detecting any change in the ephemeris representation parameters
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.4.1, p.102
    const size_t IODE;

    /// @brief Issue of Data, Clock
    ///
    /// Indicates the issue number of the data set and thereby provides the user with a convenient means of detecting any change in the
    /// subframe 1 core CEI data. Constraints on the IODC as well as the relationship between the IODC and the IODE (issue of data, ephemeris) terms
    /// are defined in \cite IS-GPS-200M GPS ICD, ch. 20.3.4.4.
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.5, p.95
    const size_t IODC;

    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.8, p.95
    const std::array<double, 3> a;

    // --------------------------------------- Keplerian Parameters ------------------------------------------

    const double sqrt_A;  ///< Square root of the semi-major axis [m^1/2]
    const double e;       ///< Eccentricity [-]
    const double i_0;     ///< Inclination angle at reference time [rad]
    const double Omega_0; ///< Longitude of the ascending node at reference time [rad]
    const double omega;   ///< Argument of perigee [rad]
    const double M_0;     ///< Mean anomaly at reference time [rad]

    // -------------------------------------- Pertubation Parameters -----------------------------------------

    const double delta_n;   ///< Mean motion difference from computed value [rad/s]
    const double Omega_dot; ///< Rate of change of right ascension [rad/s]
    const double i_dot;     ///< Rate of change of inclination [rad/s]
    const double Cus;       ///< Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    const double Cuc;       ///< Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    const double Cis;       ///< Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    const double Cic;       ///< Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    const double Crs;       ///< Amplitude of the sine harmonic correction term to the orbit radius [m]
    const double Crc;       ///< Amplitude of the cosine harmonic correction term to the orbit radius [m]

    // ----------------------------------------------- Other -------------------------------------------------

    /// @brief SV accuracy [m]
    ///
    /// Derived from an URA index of the SV for the standard positioning service user.
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.3, p.92ff
    const double svAccuracy;

    /// @brief SV health
    ///
    /// 0 = all LNAV data are OK
    /// 1 = some or all LNAV data are bad
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.4, p. 94
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.5.1.3, p. 117ff
    const uint8_t svHealth;

    /// @brief Indicate which code(s) is (are) commanded ON for the in-phase component of the L2 channel.
    ///
    /// 00 = Invalid,
    /// 01 = P-code ON,
    /// 10 = C/A-code ON,
    /// 11 = Invalid.
    ///
    /// These bits provide no indication of which code(s), if any, may be commanded ON for the quadrature component of the L2 channel.
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.2, p. 92
    const uint8_t L2ChannelCodes;

    /// @brief Data Flag for L2 P-Code
    ///
    /// 1 indicates that the LNAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel.
    /// This bit provides no indication of whether LNAV data is or is not present on any code modulated on the quadrature component of the L2 channel.
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.6, p. 95
    const bool L2DataFlagPCode;

    /// @brief Estimated Group Delay Differential. L1 and L2 correction term [s]
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.7, p. 95
    const double T_GD;

    /// @brief Fit Interval of ephemerides [h]
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.4.1, p.102
    const double fitInterval;

    // #######################################################################################################
    //                                               Functions
    // #######################################################################################################

    /// @brief Constructor
    /// @param[in] toc Time the Clock information is calculated (Time of Clock)
    /// @param[in] toe Time the Orbit information is calculated (Time of Ephemeris)
    /// @param[in] IODE Issue of Data, Ephemeris
    /// @param[in] IODC Issue of Data, Clock
    /// @param[in] a Polynomial coefficients for clock correction (a0 bias [s], a1 drift [s/s], a2 drift rate (aging) [s/s^2])
    /// @param[in] sqrt_A Square root of the semi-major axis [m^1/2]
    /// @param[in] e Eccentricity [-]
    /// @param[in] i_0 Inclination angle at reference time [rad]
    /// @param[in] Omega_0 Longitude of the ascending node at reference time [rad]
    /// @param[in] omega Argument of perigee [rad]
    /// @param[in] M_0 Mean anomaly at reference time [rad]
    /// @param[in] delta_n Mean motion difference from computed value [rad/s]
    /// @param[in] Omega_dot Rate of change of right ascension [rad/s]
    /// @param[in] i_dot Rate of change of inclination [rad/s]
    /// @param[in] Cus Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    /// @param[in] Cuc Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    /// @param[in] Cis Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Cic Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Crs Amplitude of the sine harmonic correction term to the orbit radius [m]
    /// @param[in] Crc Amplitude of the cosine harmonic correction term to the orbit radius [m]
    /// @param[in] svAccuracy SV accuracy [m]
    /// @param[in] svHealth Signal Health
    /// @param[in] L2ChannelCodes Indicate which code(s) is (are) commanded ON for the in-phase component of the L2 channel
    /// @param[in] L2DataFlagPCode Data Flag for L2 P-Code
    /// @param[in] T_GD Estimated Group Delay Differential. L1 and L2 correction term [s]
    /// @param[in] fitInterval Fit Interval of ephemerides [h]
    GPSEphemeris(const InsTime& toc, const InsTime& toe,
                 const size_t& IODE, const size_t& IODC,
                 const std::array<double, 3>& a,
                 const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                 const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                 const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                 const double& svAccuracy, uint8_t svHealth,
                 uint8_t L2ChannelCodes, bool L2DataFlagPCode,
                 const double& T_GD,
                 const double& fitInterval);

#ifdef TESTING
    /// @brief Constructor for pasting raw data from Nav files
    /// @param[in] year Time of Clock year
    /// @param[in] month Time of Clock month
    /// @param[in] day Time of Clock day
    /// @param[in] hour Time of Clock hour
    /// @param[in] minute Time of Clock minute
    /// @param[in] second Time of Clock second
    /// @param[in] svClockBias Clock correction a(0) bias [s]
    /// @param[in] svClockDrift Clock correction a(1) drift [s/s]
    /// @param[in] svClockDriftRate Clock correction a(2) drift rate (aging) [s/s^2]
    /// @param[in] IODE Issue of Data, Ephemeris
    /// @param[in] Crs Amplitude of the sine harmonic correction term to the orbit radius [m]
    /// @param[in] delta_n Mean motion difference from computed value [rad/s]
    /// @param[in] M_0 Mean anomaly at reference time [rad]
    /// @param[in] Cuc Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    /// @param[in] e Eccentricity [-]
    /// @param[in] Cus Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    /// @param[in] sqrt_A Square root of the semi-major axis [m^1/2]
    /// @param[in] Toe Time of Ephemeris
    /// @param[in] Cic Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Omega_0 Longitude of the ascending node at reference time [rad]
    /// @param[in] Cis Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    /// @param[in] i_0 Inclination angle at reference time [rad]
    /// @param[in] Crc Amplitude of the cosine harmonic correction term to the orbit radius [m]
    /// @param[in] omega Argument of perigee [rad]
    /// @param[in] Omega_dot Rate of change of right ascension [rad/s]
    /// @param[in] i_dot Rate of change of inclination [rad/s]
    /// @param[in] L2ChannelCodes Indicate which code(s) is (are) commanded ON for the in-phase component of the L2 channel.
    /// @param[in] GPSWeek GPS Week to go with Toe
    /// @param[in] L2DataFlagPCode Data Flag for L2 P-Code
    /// @param[in] svAccuracy SV accuracy [m]
    /// @param[in] svHealth SV health
    /// @param[in] T_GD Estimated Group Delay Differential. L1 and L2 correction term [s]
    /// @param[in] IODC Issue of Data, Clock
    /// @param[in] TransmissionTimeOfMessage Transmission time of message
    /// @param[in] fitInterval Fit Interval of ephemerides [h]
    /// @param[in] spare1 Spare data
    /// @param[in] spare2 Spare data
    GPSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                 double IODE, double Crs, double delta_n, double M_0,
                 double Cuc, double e, double Cus, double sqrt_A,
                 double Toe, double Cic, double Omega_0, double Cis,
                 double i_0, double Crc, double omega, double Omega_dot,
                 double i_dot, double L2ChannelCodes, double GPSWeek, double L2DataFlagPCode,
                 double svAccuracy, double svHealth, double T_GD, double IODC,
                 double TransmissionTimeOfMessage, double fitInterval, double spare1 = 0.0, double spare2 = 0.0);
#endif

    /// @brief Destructor
    ~GPSEphemeris() final = default;
    /// @brief Copy constructor
    GPSEphemeris(const GPSEphemeris&) = default;
    /// @brief Move constructor
    GPSEphemeris(GPSEphemeris&&) = default;
    /// @brief Copy assignment operator
    GPSEphemeris& operator=(const GPSEphemeris&) = delete;
    /// @brief Move assignment operator
    GPSEphemeris& operator=(GPSEphemeris&&) = delete;

    /// @brief Calculates the Variance of the satellite position in [m^2]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receive time of the signal
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

  private:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time of the signal
    /// @param[in] calc Flags which determine what should be calculated and returned
    /// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    [[nodiscard]] PosVelAccel calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const final;
};

} // namespace NAV