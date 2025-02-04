// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file QZSSEphemeris.hpp
/// @brief QZSS Ephemeris information
/// @author P. Peitschat (Hiwi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-23

#pragma once

#include <bitset>

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// @brief Broadcasted ephemeris message data
/// @note See \cite IS-QZSS-PNT-005 IS QZSS, Table 4.1.2-4, p.46 and Table 4.1.2-7, p.49
class QZSSEphemeris final : public SatNavData
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
    /// Users can detect the update of the ephemeris parameter by IODE
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.4, p.50
    const size_t IODE;

    /// @brief Issue of Data, Clock
    ///
    /// Users can detect the update of SV clock parameter by IODC
    /// 2 MSBs (most significant bit) of IODC indicate fit intervals:
    /// --------------------------------------------------
    /// | 2 MSBs of IODC(binary) | Fit intervals(minutes)
    /// --------------------------------------------------
    /// |           00           |          15
    /// |           01           |          30
    /// |           10           |          60
    /// |           11           |         120
    /// --------------------------------------------------
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.3, p.48
    const size_t IODC;

    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 5.5.1, p.130
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
    const double Omega_dot; ///< Rate of right ascension [rad/s]
    const double i_dot;     ///< Rate of inclination angle [rad/s]
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
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.3, p.47
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 5.4.3, p.125 ff.
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.3, p.92ff
    const double svAccuracy;

    /// @brief SV health
    ///
    /// This consists of the 1MSB health and the 5LSBs health
    ///
    /// 1-bit Health
    /// --------------------------------------------------
    /// Bit Location  | Name         | Target Signal
    /// --------------------------------------------------
    /// 1st bit (MSB) | L1 Health    | L1C/A or L1C/B
    /// --------------------------------------------------
    ///
    /// 5-bit Health
    /// --------------------------------------------------
    /// Bit Location  | Name         | Target Signal
    /// --------------------------------------------------
    /// 1st bit (MSB) | L1C/A Health | L1C/A
    /// 2nd bit       | L2 Health    | L2C
    /// 3rd bit       | L5 Health    | L5
    /// 4th bit       | L1C Health   | L1C
    /// 5th bit (LSB) | L1C/B Health | L1C/B
    /// --------------------------------------------------
    ///
    /// The 5-bit health parameter is defined differently from GPS. For details, see \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.7, p.61
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.3, p.47
    const std::bitset<6> svHealth;

    /// @brief Indicate which code(s) is (are) commanded ON for the in-phase component of the L2 channel.
    ///
    /// Fixed to '2'.
    ///
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.2, p.92
    /// @note See \cite RINEX-3.04, A31
    const uint8_t L2ChannelCodes;

    /// @brief Data Flag for L2 P-Code
    ///
    /// Fixed to '1' since QZSS does not track L2P.
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.3, p.48
    const bool L2DataFlagPCode;

    /// @brief Group delay between SV clock and L1C/A [s]
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 5.8, p.141 ff.
    const double T_GD;

    /// @brief Fit Interval period of the ephemeris
    ///
    /// "0": 2 hours
    /// "1": Greater than 2 hours
    ///
    /// For QZSS always fixed to "0", since fit interval period of the ephemeris is 2 hours (see Table 4.1.1-2.).
    ///
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 4.1.2.4, p.50
    const bool fitIntervalFlag;

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
    /// @param[in] svHealth SV Health
    /// @param[in] L2ChannelCodes Indicate which code(s) is (are) commanded ON for the in-phase component of the L2 channel
    /// @param[in] L2DataFlagPCode Data Flag for L2 P-Code (fixed to '1')
    /// @param[in] T_GD Group delay between SV clock and L1C/A [s]
    /// @param[in] fitIntervalFlag Fit Interval period of the ephemeris
    QZSSEphemeris(const InsTime& toc, const InsTime& toe,
                  const size_t& IODE, const size_t& IODC,
                  const std::array<double, 3>& a,
                  const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                  const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                  const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                  const double& svAccuracy, uint8_t svHealth,
                  uint8_t L2ChannelCodes, bool L2DataFlagPCode,
                  const double& T_GD, bool fitIntervalFlag);

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
    /// @param[in] fitIntervalFlag Fit Interval period of the ephemeris
    /// @param[in] spare1 Spare data
    /// @param[in] spare2 Spare data
    QZSSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                  double IODE, double Crs, double delta_n, double M_0,
                  double Cuc, double e, double Cus, double sqrt_A,
                  double Toe, double Cic, double Omega_0, double Cis,
                  double i_0, double Crc, double omega, double Omega_dot,
                  double i_dot, double L2ChannelCodes, double GPSWeek, double L2DataFlagPCode,
                  double svAccuracy, double svHealth, double T_GD, double IODC,
                  double TransmissionTimeOfMessage, double fitIntervalFlag, double spare1 = 0.0, double spare2 = 0.0);
#endif

    /// @brief Destructor
    ~QZSSEphemeris() final = default;
    /// @brief Copy Constructor
    QZSSEphemeris(const QZSSEphemeris&) = default;
    /// @brief Move Constructor
    QZSSEphemeris(QZSSEphemeris&&) = default;
    /// @brief Copy assignment operator
    QZSSEphemeris& operator=(const QZSSEphemeris&) = delete;
    /// @brief Move assignment operator
    QZSSEphemeris& operator=(QZSSEphemeris&&) = delete;

    /// @brief Calculates the Variance of the satellite position in [m^2]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receive time of the signal
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 5.5.2, p.130
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

  private:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time of the signal
    /// @param[in] calc Flags which determine what should be calculated and returned
    /// @note See \cite IS-QZSS-PNT-005 IS QZSS, ch. 5.6.1.2, p.133 ff.
    /// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    [[nodiscard]] PosVelAccel calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const final;
};

} // namespace NAV