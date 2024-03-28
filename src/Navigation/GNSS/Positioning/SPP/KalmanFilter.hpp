// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KalmanFilter.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author P. Peitschat (paula.peitschat@ins.uni-stuttgart.de)
/// @date 2023-12-22

#pragma once

#include <array>
#include <set>

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"
#include "Navigation/Time/InsTime.hpp"

#include "util/Eigen.hpp"

namespace NAV::SPP
{

/// @brief The Spp Kalman Filter related options
class KalmanFilter // NOLINT(clang-analyzer-optin.performance.Padding)
{
  public:
    /// @brief Resets the filter
    void reset();

    /// @brief Initialize the filter
    /// @param states States to initialize with
    /// @param variance Variance of the state
    void initialize(const KeyedVectorXd<States::StateKeyTypes>& states, const KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>& variance);

    /// @brief Deinitialize the KF (can be used to reinitialize the Filter when results seem strange)
    void deinitialize();

    /// @brief Checks wether the Kalman filter is initialized
    [[nodiscard]] bool isInitialized() const { return _initialized; }

    /// @brief Does the Kalman Filter prediction
    /// @param[in] dt Time step [s]
    /// @param[in] lla_pos Position in Latitude, Longitude, Altitude [rad, rad, m]
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void predict(const double& dt, const Eigen::Vector3d& lla_pos, const std::string& nameId);

    /// @brief Does the Kalman Filter update
    /// @param[in] measKeys Measurement keys
    /// @param[in] H Measurement sensitivity matrix 𝐇
    /// @param[in] R Measurement noise covariance matrix 𝐑
    /// @param[in] dz Measurement innovation 𝜹𝐳
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    void update(const std::vector<Meas::MeasKeyTypes>& measKeys,
                const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>& H,
                const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& R,
                const KeyedVectorXd<Meas::MeasKeyTypes>& dz,
                const std::string& nameId);

    /// @brief Adds the inter system time difference bias and drift
    /// @param[in] usedSatSystems Used Satellite systems this epoch
    /// @param[in] oldRefSys Old Satellite time reference system
    /// @param[in] newRefSys New Satellite time reference system
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @return The reference system which was selected
    SatelliteSystem updateInterSystemTimeDifferences(const std::set<SatelliteSystem>& usedSatSystems,
                                                     SatelliteSystem oldRefSys,
                                                     SatelliteSystem newRefSys,
                                                     const std::string& nameId);

    /// @brief Adds the frequency as inter-frequency bias state
    /// @param[in] freq Frequency to estimate the inter-frequency bias for
    void addInterFrequencyBias(const Frequency& freq);

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time step [s]
    /// @param[in] lla_pos Position in Latitude, Longitude, Altitude [rad, rad, m]
    /// @param[in] nameId Name and id of the node calling this (only used for logging purposes)
    /// @note See \cite Groves2013 Groves, ch. 9.4.2.2, eq. 9.152, p. 417
    KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>
        calcProcessNoiseMatrixGroves(double dt, const Eigen::Vector3d& lla_pos, const std::string& nameId) const;

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in] useDoppler Whether to use doppler measurements
    /// @param[in] multiConstellation Whether to use multiple constellations
    /// @param[in] estimateInterFrequencyBiases Whether to use estimate inter frequency biases
    /// @param[in] itemWidth Width of the widgets
    /// @param[in] unitWidth  Width on unit inputs
    /// @return True when something was changed
    bool ShowGuiWidgets(const char* id, bool useDoppler, bool multiConstellation, bool estimateInterFrequencyBiases, float itemWidth, float unitWidth);

    /// @brief Set the P matrix entry for the covariance of the clock phase drift
    /// @param clkPhaseDrift Clock phase drift variance in [m^2 / s]
    void setClockBiasErrorCovariance(double clkPhaseDrift);

    /// @brief Get the States in the Kalman Filter
    [[nodiscard]] const std::vector<SPP::States::StateKeyTypes>& getStateKeys() const;

    /// @brief Returns the State vector x̂
    [[nodiscard]] const KeyedVectorXd<States::StateKeyTypes>& getState() const;

    /// @brief Returns the Error covariance matrix 𝐏
    [[nodiscard]] const KeyedMatrixXd<States::StateKeyTypes, States::StateKeyTypes>& getErrorCovarianceMatrix() const;

  private:
    /// Kalman Filter representation
    KeyedKalmanFilterD<SPP::States::StateKeyTypes, SPP::Meas::MeasKeyTypes> _kalmanFilter{ SPP::States::PosVelRecvClk, {} };

    /// Boolean that determines, if Kalman Filter is initialized (from weighted LSE solution)
    bool _initialized = false;

    // ###########################################################################################################
    //                                               GUI settings
    // ###########################################################################################################

    /// GUI option for the Q calculation algorithm
    enum class QCalculationAlgorithm
    {
        VanLoan, ///< Van-Loan
        Taylor1, ///< Taylor
    };
    /// Q calculation algorithm
    QCalculationAlgorithm _qCalculationAlgorithm = QCalculationAlgorithm::VanLoan;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the acceleration due to user motion
    enum class CovarianceAccelUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input covarianceAccel parameter for the StDev due to acceleration due to user motion
    CovarianceAccelUnits _gui_covarianceAccelUnit = CovarianceAccelUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_covarianceAccel = { 3.0, 1.5 } /* [ m / √(s^3) ] */;
    /// @brief Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    std::array<double, 2> _covarianceAccel = { 3.0, 1.5 };

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock phase drift
    enum class CovarianceClkPhaseDriftUnits
    {
        m_sqrts, ///< [ m / √(s) ]
        m2_s,    ///< [ m^2 / s ]
    };
    /// Gui selection for the Unit of the input covarianceClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits _gui_covarianceClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m2_s;

    /// @brief GUI selection for the Standard deviation of the clock phase drift
    double _gui_covarianceClkPhaseDrift = 0.01 /*[ m^2 / s ] */;
    /// @brief Covariance of the clock phase drift [m²/s]
    double _covarianceClkPhaseDrift = 0.01;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock frequency drift
    enum class CovarianceClkFrequencyDriftUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input covarianceClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits _gui_covarianceClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m2_s3;

    /// @brief GUI selection for the Standard deviation of the clock frequency drift
    double _gui_covarianceClkFrequencyDrift = 0.04 /* [ m^2 / s^3 ] */;
    /// @brief Covariance of the clock frequency drift [m²/s³]
    double _covarianceClkFrequencyDrift = 0.04;

    // ###########################################################################################################

    /// Gui selection for the Unit of the inter system covarianceInterSysClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits _gui_covarianceInterSysClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m2_s;

    /// @brief GUI selection for the Standard deviation of the inter-system clock phase drift
    double _gui_covarianceInterSysClkPhaseDrift = 0.01 /* [m²/s] */;
    /// @brief Covariance of the inter-system clock phase drift [m²/s]
    double _covarianceInterSysClkPhaseDrift = 0.01;

    // ###########################################################################################################

    /// Gui selection for the Unit of the inter system covarianceInterSysClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits _gui_covarianceInterSysClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m2_s3;

    /// @brief GUI selection for the Standard deviation of the inter-system clock frequency drift
    double _gui_covarianceInterSysClkFrequencyDrift = 0.04 /* [m²/s³] */;
    /// @brief Covariance of the inter-system clock frequency drift [m²/s³]
    double _covarianceInterSysClkFrequencyDrift = 0.04;

    // ###########################################################################################################

    /// Gui selection for the Unit of the inter-frequency covarianceInterFrequencyBias parameter
    CovarianceClkPhaseDriftUnits _gui_covarianceInterFrequencyBiasUnit = CovarianceClkPhaseDriftUnits::m2_s;

    /// @brief GUI selection for the Standard deviation of the inter-frequency bias
    double _gui_covarianceInterFrequencyBias = 1e-6 /* [m²/s] */;
    /// @brief Covariance of the inter-frequency bias [m²/s]
    double _covarianceInterFrequencyBias = 1e-6;

    // ###########################################################################################################
    // ###########################################################################################################

    /// Possible Units for the P matrix initialization velocity uncertainty
    enum class InitCovarianceVelocityUnits
    {
        m_s,   ///< [ m / s ]
        m2_s2, ///< [ m^2 / s^2 ]
    };
    /// Gui selection for the Unit of the P matrix initialization velocity uncertainty
    InitCovarianceVelocityUnits _gui_initCovarianceVelocityUnit = InitCovarianceVelocityUnits::m_s;

    /// @brief GUI selection for the P matrix initialization velocity uncertainty
    double _gui_initCovarianceVelocity = 10 /* [ m / s ] */;
    /// @brief Covariance of the P matrix initialization velocity uncertainty [m²/s²]
    double _initCovarianceVelocity = 100;

    // ###########################################################################################################

    /// Possible Units for the P matrix initialization clock drift uncertainty
    enum class InitCovarianceClockDriftUnits
    {
        m_s,   ///< [ m / s ]
        s_s,   ///< [ s / s ]
        m2_s2, ///< [ m^2 / s^2 ]
        s2_s2, ///< [ s^2 / s^2 ]
    };
    /// Gui selection for the Unit of the P matrix initialization clock drift uncertainty
    InitCovarianceClockDriftUnits _gui_initCovarianceClockDriftUnit = InitCovarianceClockDriftUnits::s_s;

    /// @brief GUI selection for the P matrix initialization clock drift uncertainty
    double _gui_initCovarianceClockDrift = 1e-6 /* [ s / s ] */;
    /// @brief Covariance of the P matrix initialization clock drift uncertainty [m²/s²]
    double _initCovarianceClockDrift = std::pow(1e-6 * InsConst<>::C, 2);

    // ###########################################################################################################

    /// Gui selection for the Unit of the P matrix initialization inter system clock drift uncertainty
    InitCovarianceClockDriftUnits _gui_initCovarianceInterSysClockDriftUnit = InitCovarianceClockDriftUnits::s_s;

    /// @brief GUI selection for the P matrix initialization inter system clock drift uncertainty
    double _gui_initCovarianceInterSysClockDrift = 1e-6 /* [ s / s ] */;
    /// @brief Covariance of the P matrix initialization inter system clock drift uncertainty [m²/s²]
    double _initCovarianceInterSysClockDrift = std::pow(1e-6 * InsConst<>::C, 2);

    // ###########################################################################################################
    // ###########################################################################################################

    friend void to_json(json& j, const KalmanFilter& data);
    friend void from_json(const json& j, KalmanFilter& data);
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const KalmanFilter& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, KalmanFilter& data);

} // namespace NAV::SPP
