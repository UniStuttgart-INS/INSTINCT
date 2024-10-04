// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file BDSEphemeris.hpp
/// @brief BDS Ephemeris information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-02

#pragma once

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

#include "Navigation/Time/InsTime.hpp"

#include <set>

namespace NAV
{

// /// @brief List of GEO Satellites of BDS
// const static std::set<uint16_t> GeoSats{ 1, 2, 3, 4, 5 };

/// @brief Broadcasted ephemeris message data
/// @note See \cite BDS-SIS-ICD-2.1 BDS-SIS-ICD-2.1, ch. 5.2.4, p. 23ff
class BDSEphemeris final : public SatNavData
{
  public:
    // #######################################################################################################
    //                                                Members
    // #######################################################################################################

    /// @brief Number of the satellite
    const uint16_t satNum;

    // ------------------------------------------ Time Parameters --------------------------------------------

    /// @brief Time of Clock
    const InsTime toc;

    /// @brief Time of Ephemeris
    const InsTime toe;

    /// @brief Age of Data, Ephemeris
    ///
    /// Age of data, ephemeris (AODE) is the extrapolated interval of ephemeris parameters. It indicates the time difference between the
    /// reference epoch of ephemeris parameters and the last observation epoch for extrapolating ephemeris parameters. AODE is updated at the start of
    /// each hour in BDT, and it is 5 bits long with definitions as follows:
    ///
    /// | AODE | Definition                                                   |
    /// | :-:  | ---                                                          |
    /// | < 25 | Age of the satellite ephemeris parameters in hours           |
    /// |  25  | Age of the satellite ephemeris parameters is two days        |
    /// |  26  | Age of the satellite ephemeris parameters is three days      |
    /// |  27  | Age of the satellite ephemeris parameters is four days       |
    /// |  28  | Age of the satellite ephemeris parameters is five days       |
    /// |  29  | Age of the satellite ephemeris parameters is six days        |
    /// |  30  | Age of the satellite ephemeris parameters is seven days      |
    /// |  31  | Age of the satellite ephemeris parameters is over seven days |
    ///
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.11, Table 5-8, p. 31f
    const size_t AODE;

    /// @brief Age of Data, Clock
    ///
    /// Age of data, clock (AODC) is the extrapolated interval of clock correction parameters. It indicates the time difference between the
    /// reference epoch of clock correction parameters and the last observation epoch for extrapolating clock correction parameters.
    /// AODC is updated at the start of each hour in BDT, and it is 5 bits long with definitions as
    ///
    /// | AODC | Definition                                                          |
    /// | :-:  | ---                                                                 |
    /// | < 25 | Age of the satellite clock correction parameters in hours           |
    /// |  25  | Age of the satellite clock correction parameters is two days        |
    /// |  26  | Age of the satellite clock correction parameters is three days      |
    /// |  27  | Age of the satellite clock correction parameters is four days       |
    /// |  28  | Age of the satellite clock correction parameters is five days       |
    /// |  29  | Age of the satellite clock correction parameters is six days        |
    /// |  30  | Age of the satellite clock correction parameters is seven days      |
    /// |  31  | Age of the satellite clock correction parameters is over seven days |
    ///
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.9, Table 5-6, p. 28f
    const size_t AODC;

    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    ///
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.10, p. 29f
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
    /// Derived from an URA index (URAI) of the SV for the standard positioning service user.
    ///
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.5, p. 24
    const double svAccuracy;

    /// @brief Autonomous Satellite Health flag
    ///
    /// 0 = broadcasting satellite is good
    /// 1 = broadcasting satellite is not good
    ///
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.6, p. 25
    const uint8_t satH1;

    /// @brief Equipment Group Delay Differential. B1/B3 [s]
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.8, p. 28
    const double T_GD1;

    /// @brief Equipment Group Delay Differential. B2/B3 [s]
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.8, p. 28
    const double T_GD2;

    // #######################################################################################################
    //                                               Functions
    // #######################################################################################################

    /// @brief Constructor
    /// @param[in] satNum Number of the satellite
    /// @param[in] toc Time the Clock information is calculated (Time of Clock)
    /// @param[in] toe Time the Orbit information is calculated (Time of Ephemeris)
    /// @param[in] AODE Age of Data, Ephemeris
    /// @param[in] AODC Age of Data, Clock
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
    /// @param[in] satH1 Autonomous Satellite Health flag
    /// @param[in] T_GD1 Equipment Group Delay Differential. B1/B3 [s]
    /// @param[in] T_GD2 Equipment Group Delay Differential. B2/B3 [s]
    BDSEphemeris(const uint16_t& satNum, const InsTime& toc, const InsTime& toe,
                 const size_t& AODE, const size_t& AODC,
                 const std::array<double, 3>& a,
                 const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                 const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                 const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                 const double& svAccuracy, uint8_t satH1, double T_GD1, double T_GD2);

#ifdef TESTING
    /// @brief Constructor for pasting raw data from Nav files
    /// @param[in] satNum Number of the satellite
    /// @param[in] year Time of Clock year
    /// @param[in] month Time of Clock month
    /// @param[in] day Time of Clock day
    /// @param[in] hour Time of Clock hour
    /// @param[in] minute Time of Clock minute
    /// @param[in] second Time of Clock second
    /// @param[in] svClockBias Clock correction a(0) bias [s]
    /// @param[in] svClockDrift Clock correction a(1) drift [s/s]
    /// @param[in] svClockDriftRate Clock correction a(2) drift rate (aging) [s/s^2]
    /// @param[in] AODE Age of Data, Ephemeris
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
    /// @param[in] spare1 Spare data
    /// @param[in] BDTWeek BDT Week to go with Toe (not GPS Week)
    /// @param[in] spare2 Spare data
    /// @param[in] svAccuracy SV accuracy [m]
    /// @param[in] satH1 Autonomous Satellite Health flag
    /// @param[in] T_GD1 Equipment Group Delay Differential. B1/B3 [s]
    /// @param[in] T_GD2 Equipment Group Delay Differential. B2/B3 [s]
    /// @param[in] TransmissionTimeOfMessage Transmission time of message
    /// @param[in] AODC Age of Data, Clock
    /// @param[in] spare3 Spare data
    /// @param[in] spare4 Spare data
    BDSEphemeris(int32_t satNum, int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                 double AODE, double Crs, double delta_n, double M_0,
                 double Cuc, double e, double Cus, double sqrt_A,
                 double Toe, double Cic, double Omega_0, double Cis,
                 double i_0, double Crc, double omega, double Omega_dot,
                 double i_dot, double spare1, double BDTWeek, double spare2,
                 double svAccuracy, double satH1, double T_GD1, double T_GD2,
                 double TransmissionTimeOfMessage, double AODC, double spare3 = 0.0, double spare4 = 0.0);
#endif

    /// @brief Destructor
    ~BDSEphemeris() final = default;
    /// @brief Copy constructor
    BDSEphemeris(const BDSEphemeris&) = default;
    /// @brief Move constructor
    BDSEphemeris(BDSEphemeris&&) = default;
    /// @brief Copy assignment operator
    BDSEphemeris& operator=(const BDSEphemeris&) = delete;
    /// @brief Move assignment operator
    BDSEphemeris& operator=(BDSEphemeris&&) = delete;

    /// @brief Calculates the Variance of the satellite position in [m^2]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receive time of the signal
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.10, p. 29ff
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

  private:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time of the signal
    /// @param[in] calc Flags which determine what should be calculated and returned
    /// @note See \cite BDS-SIS-ICD-2.1 BDS ICD, ch. 5.2.4.12, p. 32ff
    /// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    [[nodiscard]] PosVelAccel calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const final;
};

} // namespace NAV