// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GalileoEphemeris.hpp
/// @brief Galileo Ephemeris information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-12

#pragma once

#include <bitset>

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

namespace NAV
{

/// @brief Broadcasted ephemeris message data
/// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5., p. 43ff
///
/// \image html GPS-satellite-orbits.png "Description of the GPS satellite orbits"
class GalileoEphemeris final : public SatNavData
{
  public:
    /// @brief Navigation Data Validity and Signal Health Status
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.9.3, p. 52
    struct SvHealth
    {
        /// @brief Navigation Data Validity
        enum DataValidityStatus
        {
            NavigationDataValid = 0,     ///< Navigation data valid
            WorkingWithoutGuarantee = 1, ///< Working without guarantee
        };
        /// @brief Signal Health Status
        enum SignalHealthStatus
        {
            SignalOK = 0,                       ///< Signal OK
            SignalOutOfService = 1,             ///< Signal out of service
            SignalWillBeOutOfService = 2,       ///< Signal will be out of service
            SignalComponentCurrentlyInTest = 3, ///< Signal Component currently in Test
        };

        DataValidityStatus E5a_DataValidityStatus; ///< E5a Data Validity Status
        DataValidityStatus E5b_DataValidityStatus; ///< E5b Data Validity Status
        DataValidityStatus E1B_DataValidityStatus; ///< E1-B Data Validity Status

        SignalHealthStatus E5a_SignalHealthStatus;  ///< E5a Signal Health Status
        SignalHealthStatus E5b_SignalHealthStatus;  ///< E5b Signal Health Status
        SignalHealthStatus E1BC_SignalHealthStatus; ///< E1-B/C Signal Health Status
    };

    // #######################################################################################################
    //                                                Members
    // #######################################################################################################

    // ------------------------------------------ Time Parameters --------------------------------------------

    /// @brief Time of Clock
    const InsTime toc;

    /// @brief Time of Ephemeris
    const InsTime toe;

    /// @brief Issue of Data of the nav batch
    ///
    /// The navigation data is disseminated in data batches each one identified by an Issue of
    /// Data. In nominal operation the navigation data (ephemeris, satellite clock correction and
    /// SISA) have limited validity duration depending on the data type. The identification of each
    /// batch by an Issue of Data (IOD) value enables:
    /// - the users to distinguish the data in different batches received from each satellite
    /// - to indicate to the user receiver the validity of the data (which have to be updated
    ///   using new issue of navigation data)
    /// - the user receiver to compute the full batch of data even if it misses some pages
    ///   or start receiving the data somewhere during the transmission
    ///
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.9.2, p. 51
    const size_t IODnav;

    /// Polynomial coefficients for clock correction
    /// - a(0) bias [s]
    /// - a(1) drift [s/s]
    /// - a(2) drift rate (aging) [s/s^2]
    ///
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.3, p. 46
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

    /// @brief Data sources
    ///
    /// Bit 0 set: I/NAV E1-B
    /// Bit 1 set: F/NAV E5a-I
    /// Bit 2 set: I/NAV E5b-I
    /// Bits 0 and 2: Both can be set if the navigation messages were merged, however, bits 0-2 cannot all be set,
    ///               as the I/NAV and F/NAV messages contain different information
    /// Bit 3 reserved for Galileo internal use
    /// Bit 4 reserved for Galileo internal use
    /// Bit 8 set: af0-af2, Toc, SISA are for E5a,E1
    /// Bit 9 set: af0-af2, Toc, SISA are for E5b,E1
    /// Bits 8-9 : exclusive (only one bit can be set)
    ///
    /// @note See \cite RINEX-3.04 RINEX-3.04, ch. A8, p. A25
    const std::bitset<10> dataSource;

    /// SISA (Signal in space accuracy) [m]
    ///
    /// Signal – In – Space Accuracy (SISA) is a prediction of the minimum standard deviation
    /// (1-sigma) of the unbiased Gaussian distribution which overbounds the Signal – In – Space
    /// Error (SISE) predictable distribution for all possible user locations within the satellite
    /// coverage area. When no accurate prediction is available (SISA = NAPA), this is an indicator
    /// of a potential anomalous SIS.
    ///
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.12, p.58
    const double signalAccuracy;

    /// @brief Signal Health
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.9.3, p. 52
    const SvHealth svHealth;

    /// @brief E1-E5a Broadcast Group Delay [s]
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.5, p. 47f
    const double BGD_E1_E5a;
    /// @brief E1-E5b Broadcast Group Delay [s]
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.5, p. 47f
    const double BGD_E1_E5b;

    // #######################################################################################################
    //                                               Functions
    // #######################################################################################################

    /// @brief Constructor
    /// @param[in] toc Time the Clock information is calculated (Time of Clock)
    /// @param[in] toe Time the Orbit information is calculated (Time of Ephemeris)
    /// @param[in] IODnav Issue of Data of the nav batch
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
    /// @param[in] dataSource Data sources
    /// @param[in] signalAccuracy SISA (Signal in space accuracy) [m]
    /// @param[in] svHealth Signal Health
    /// @param[in] BGD_E1_E5a E1-E5a Broadcast Group Delay [s]
    /// @param[in] BGD_E1_E5b E1-E5b Broadcast Group Delay [s]
    GalileoEphemeris(const InsTime& toc, const InsTime& toe,
                     const size_t& IODnav,
                     const std::array<double, 3>& a,
                     const double& sqrt_A, const double& e, const double& i_0, const double& Omega_0, const double& omega, const double& M_0,
                     const double& delta_n, const double& Omega_dot, const double& i_dot, const double& Cus, const double& Cuc,
                     const double& Cis, const double& Cic, const double& Crs, const double& Crc,
                     const std::bitset<10>& dataSource, const double& signalAccuracy, const SvHealth& svHealth,
                     const double& BGD_E1_E5a, const double& BGD_E1_E5b);

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
    /// @param[in] IODnav Issue of Data of the nav batch
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
    /// @param[in] dataSource Data sources
    /// @param[in] GALWeek GAL Week to go with Toe
    /// @param[in] spare1 Spare data
    /// @param[in] signalAccuracy SISA Signal in space accuracy [m]
    /// @param[in] svHealth SV health
    /// @param[in] BGD_E1_E5a E1-E5a Broadcast Group Delay [s]
    /// @param[in] BGD_E1_E5b E1-E5b Broadcast Group Delay [s]
    /// @param[in] TransmissionTimeOfMessage Transmission time of message
    /// @param[in] spare2 Spare data
    /// @param[in] spare3 Spare data
    /// @param[in] spare4 Spare data
    GalileoEphemeris(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, double second, double svClockBias, double svClockDrift, double svClockDriftRate,
                     double IODnav, double Crs, double delta_n, double M_0,
                     double Cuc, double e, double Cus, double sqrt_A,
                     double Toe, double Cic, double Omega_0, double Cis,
                     double i_0, double Crc, double omega, double Omega_dot,
                     double i_dot, double dataSource, double GALWeek, double spare1,
                     double signalAccuracy, double svHealth, double BGD_E1_E5a, double BGD_E1_E5b,
                     double TransmissionTimeOfMessage, double spare2 = 0.0, double spare3 = 0.0, double spare4 = 0.0);
#endif

    /// @brief Destructor
    ~GalileoEphemeris() final = default;
    /// @brief Copy constructor
    GalileoEphemeris(const GalileoEphemeris&) = default;
    /// @brief Move constructor
    GalileoEphemeris(GalileoEphemeris&&) = default;
    /// @brief Copy assignment operator
    GalileoEphemeris& operator=(const GalileoEphemeris&) = delete;
    /// @brief Move assignment operator
    GalileoEphemeris& operator=(GalileoEphemeris&&) = delete;

    /// @brief Calculates the Variance of the satellite position in [m]
    [[nodiscard]] double calcSatellitePositionVariance() const final;

    /// @brief Calculates clock bias and drift of the satellite
    /// @param[in] recvTime Receiver time to calculate the satellite position for
    /// @param[in] dist Distance between receiver and satellite (normally the pseudorange) [m]
    /// @param[in] freq Signal Frequency
    [[nodiscard]] Corrections calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const final;

    /// @brief Checks whether the signal is healthy
    [[nodiscard]] bool isHealthy() const final;

  private:
    /// @brief Calculates position, velocity and acceleration of the satellite at transmission time
    /// @param[in] transTime Transmit time of the signal
    /// @param[in] calc Flags which determine what should be calculated and returned
    /// @note See \cite GAL-ICD-2.0 GAL-ICD-2.0, ch. 5.1.1, p. 44f
    /// @note See \cite IS-GPS-200M IS-GPS-200 ch. 20.3.3.4.3.1 Table 20-IV p.106-109
    [[nodiscard]] PosVelAccel calcSatelliteData(const InsTime& transTime, Orbit::Calc calc) const final;
};

} // namespace NAV