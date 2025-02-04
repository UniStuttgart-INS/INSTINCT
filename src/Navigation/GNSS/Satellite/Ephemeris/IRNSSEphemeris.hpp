// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file IRNSSEphemeris.hpp
/// @brief IRNSS Ephemeris information
/// @author P. Peitschat (Hiwi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-24

#pragma once

#include <bitset>

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// @brief Broadcasted ephemeris message data
/// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, Table 11, p.19 and Table 12, p.20
class IRNSSEphemeris final : public SatNavData
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

    /// @brief Issue of Data for Ephemeris and Clock
    ///
    /// 8-bit number which indicates the issue number of the data set.
    /// Provides the user with a convenient means of detecting any change in the ephemeris and clock parameters.
    ///
    /// The IODEC values can be found according to the following table:
    /// -----------------------------------------------------------------------------------------------------------------------------
    /// IODEC range      | Update   | Description
    /// -----------------------------------------------------------------------------------------------------------------------------
    /// 0 to 11(12 sets) | 2 hours  | Nominal set for current day, uplinked in advance with broadcast ephemeris and clock parameters
    /// 12 to 23         | 2 hours  | For update to nominal sets
    /// 24 to 29         | 4 hours  | For the ephemeris and clock parameters with update rate greater than 2 hours
    /// 30 to 75         | ----- reserved for future -----
    /// 76 to 159        | 2 hours  | During AutoNav mode of operation, IODEC values are in the range .
    /// 160 to 255       | < 30 min | For the ephemeris and clock parameters with update rate lesser than 30 minutes
    /// -----------------------------------------------------------------------------------------------------------------------------

    ///
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.3, p.29 f.
    const size_t IODEC;

    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    ///
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.2, p.29
    const std::array<double, 3> a;

    // --------------------------------------- Keplerian Parameters ------------------------------------------

    const double sqrt_A;  ///< Square root of the semi-major axis [m^1/2]
    const double e;       ///< Eccentricity [-]
    const double i_0;     ///< Inclination angle at reference time [rad]
    const double Omega_0; ///< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [rad]
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
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.4, p.30
    /// @note See \cite IS-GPS-200M GPS ICD, ch. 20.3.3.3.1.3, p.92ff
    const double svAccuracy;

    /// @brief SV health
    ///
    /// There are two one-bit health flags. Each one-bit health flag indicates the health
    /// of NAV data on L5 and S signals as follows:
    /// -----------------------------------------------------------------------
    /// Flag    | Value | Description
    /// -----------------------------------------------------------------------
    /// L5 flag |   0   | All navigation data on L5 SPS signal are OK
    ///         |   1   | Some or all navigation data on L5 SPS signal are bad
    /// -----------------------------------------------------------------------
    /// S flag  |   0   | All navigation data on S SPS signal are OK
    ///         |   1   | Some or all navigation data on S SPS signal are bad
    /// -----------------------------------------------------------------------
    ///
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.6, p.31
    const std::bitset<2> svHealth;

    /// @brief Total Group Delay
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, ch. 6.2.1.5, p.31
    const double T_GD;

    // #######################################################################################################
    //                                               Functions
    // #######################################################################################################

    /// @brief Constructor
    /// @param[in] toc Time the Clock information is calculated (Time of Clock)
    /// @param[in] toe Time the Orbit information is calculated (Time of Ephemeris)
    /// @param[in] IODEC Issue of Data for Ephemeris and Clock
    /// @param[in] a Polynomial coefficients for clock correction (a0 bias [s], a1 drift [s/s], a2 drift rate (aging) [s/s^2])
    /// @param[in] sqrt_A Square root of the semi-major axis [m^1/2]
    /// @param[in] e Eccentricity [-]
    /// @param[in] i_0 Inclination angle at reference time [rad]
    /// @param[in] Omega_0 Longitude of Ascending Node of Orbit Plane at Weekly Epoch [rad]
    /// @param[in] omega Argument of perigee [rad]
    /// @param[in] M_0 Mean anomaly at reference time [rad]
    /// @param[in] delta_n Mean motion difference from computed value [rad/s]
    /// @param[in] Omega_dot Rate of right ascension [rad/s]
    /// @param[in] i_dot Rate of inclination angle [rad/s]
    /// @param[in] Cus Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    /// @param[in] Cuc Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    /// @param[in] Cis Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Cic Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Crs Amplitude of the sine harmonic correction term to the orbit radius [m]
    /// @param[in] Crc Amplitude of the cosine harmonic correction term to the orbit radius [m]
    /// @param[in] svAccuracy SV accuracy [m]
    /// @param[in] svHealth Signal Health
    /// @param[in] T_GD Total Group Delay [s]
    IRNSSEphemeris(const InsTime& toc, const InsTime& toe,
                   const size_t& IODEC,
                   const std::array<double, 3>& a,
                   const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                   const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                   const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                   const double& svAccuracy, uint8_t svHealth,
                   const double& T_GD);

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
    /// @param[in] IODEC Issue of Data for Ephemeris and Clock
    /// @param[in] Crs Amplitude of the sine harmonic correction term to the orbit radius [m]
    /// @param[in] delta_n Mean motion difference from computed value [rad/s]
    /// @param[in] M_0 Mean anomaly at reference time [rad]
    /// @param[in] Cuc Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    /// @param[in] e Eccentricity [-]
    /// @param[in] Cus Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    /// @param[in] sqrt_A Square root of the semi-major axis [m^1/2]
    /// @param[in] Toe Time of Ephemeris
    /// @param[in] Cic Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    /// @param[in] Omega_0 Longitude of Ascending Node of Orbit Plane at Weekly Epoch [rad]
    /// @param[in] Cis Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    /// @param[in] i_0 Inclination angle at reference time [rad]
    /// @param[in] Crc Amplitude of the cosine harmonic correction term to the orbit radius [m]
    /// @param[in] omega Argument of perigee [rad]
    /// @param[in] Omega_dot Rate of right ascension [rad/s]
    /// @param[in] i_dot Rate of inclination angle [rad/s]
    /// @param[in] spare1 Spare data
    /// @param[in] IRNWeek IRN Week to go with Toe
    /// @param[in] spare2 Spare data
    /// @param[in] svAccuracy SV accuracy [m]
    /// @param[in] svHealth SV health
    /// @param[in] T_GD Total Group Delay [s]
    /// @param[in] spare3 Spare data
    /// @param[in] TransmissionTimeOfMessage Transmission time of message [s of IRNSS week]
    /// @param[in] spare4 Spare data
    /// @param[in] spare5 Spare data
    /// @param[in] spare6 Spare data
    IRNSSEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                   double IODEC, double Crs, double delta_n, double M_0,
                   double Cuc, double e, double Cus, double sqrt_A,
                   double Toe, double Cic, double Omega_0, double Cis,
                   double i_0, double Crc, double omega, double Omega_dot,
                   double i_dot, double spare1, double IRNWeek, double spare2,
                   double svAccuracy, double svHealth, double T_GD, double spare3,
                   double TransmissionTimeOfMessage, double spare4 = 0.0, double spare5 = 0.0, double spare6 = 0.0);
#endif

    /// @brief Destructor
    ~IRNSSEphemeris() final = default;
    /// @brief Copy constructor
    IRNSSEphemeris(const IRNSSEphemeris&) = default;
    /// @brief Move constructor
    IRNSSEphemeris(IRNSSEphemeris&&) = default;
    /// @brief Copy assignment operator
    IRNSSEphemeris& operator=(const IRNSSEphemeris&) = delete;
    /// @brief Move assignment operator
    IRNSSEphemeris& operator=(IRNSSEphemeris&&) = delete;

    /// @brief Calculates the Variance of the satellite position in [m^2]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receive time of the signal
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, Appendix A, p.43
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

  private:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time of the signal
    /// @param[in] calc Flags which determine what should be calculated and returned
    /// @note See \cite IRNSS-SIS-ICD-1.1 IRNSS ICD, Appendix B, p.45 ff.
    /// @note See \cite IS-GPS-200M IS-GPS-200, ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    [[nodiscard]] PosVelAccel calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const final;
};

} // namespace NAV