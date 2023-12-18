// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppKalmanFilter.hpp
/// @brief SPP with a Kalman Filter
/// @author P. Peitschat (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-19

#pragma once

#include <set>

#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SPP/SppCommon.hpp"
#include "Navigation/GNSS/Positioning/SppAlgorithmTypes.hpp"
#include "Navigation/GNSS/SNRMask.hpp"

#include "util/Eigen.hpp"

namespace NAV::GNSS::Positioning::SPP
{

/// @brief The Spp Kalman Filter related options
class SppKalmanFilter // NOLINT(clang-analyzer-optin.performance.Padding)
{
  public:
    /// Possible Units for the initial covariance for the position (standard deviation σ or Variance σ²)
    enum class InitCovariancePositionUnits
    {
        m2, ///< Variance ECEF [m², m², m²]
        m,  ///< Standard deviation ECEF [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    InitCovariancePositionUnits gui_initCovariancePositionUnit = InitCovariancePositionUnits::m;

    /// @brief GUI selection of the initial covariance diagonal values for position (standard deviation σ or Variance σ²)
    Eigen::Vector3d gui_initCovariancePosition{ 10, 10, 10 };
    /// @brief Initial covariance diagonal values for position (Variance σ²)
    Eigen::Vector3d _initCovariancePosition{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the velocity (standard deviation σ or Variance σ²)
    enum class InitCovarianceVelocityUnits
    {
        m2_s2, ///< Variance [m²/s²]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance for the velocity
    InitCovarianceVelocityUnits gui_initCovarianceVelocityUnit = InitCovarianceVelocityUnits::m_s;

    /// @brief GUI selection of the initial covariance diagonal values for velocity (standard deviation σ or Variance σ²)
    Eigen::Vector3d gui_initCovarianceVelocity{ 3, 3, 3 };
    /// @brief Initial covariance diagonal values for velocity (Variance σ²)
    Eigen::Vector3d _initCovarianceVelocity{ 3, 3, 3 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock error (standard deviation σ or Variance σ²)
    enum class InitCovarianceRecvClkErrUnits
    {
        s2, ///< Variance [s²]
        s,  ///< Standard deviation [s]
    };
    /// Gui selection for the Unit of the initial covariance for the receiver clock error
    InitCovarianceRecvClkErrUnits gui_initCovarianceRecvClkErrUnit = InitCovarianceRecvClkErrUnits::s;

    /// @brief GUI selection of the initial covariance for the receiver clock error (standard deviation σ or Variance σ²)
    double gui_initCovarianceRecvClkErr = 1e-8;
    /// @brief Initial covariance for the receiver clock error (Variance σ²)
    double _initCovarianceRecvClkErr = 1e-8;

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the receiver clock drift (standard deviation σ or Variance σ²)
    enum class InitCovarianceRecvClkDriftUnits
    {
        s2_s2, ///< Variance [s²/s²]
        s_s,   ///< Standard deviation [s/s]
    };
    /// Gui selection for the Unit of the initial covariance for the receiver clock drift
    InitCovarianceRecvClkDriftUnits gui_initCovarianceRecvClkDriftUnit = InitCovarianceRecvClkDriftUnits::s_s;

    /// @brief GUI selection of the initial covariance for the receiver clock drift (standard deviation σ or Variance σ²)
    double gui_initCovarianceRecvClkDrift = 1e-10;
    /// @brief Initial covariance for the receiver clock drift (Variance σ²)
    double _initCovarianceRecvClkDrift = 1e-10;

    // ###########################################################################################################

    /// Gui selection for the Unit of the initial covariance for the inter-system clock error
    InitCovarianceRecvClkErrUnits gui_initCovarianceInterSysErrUnit = InitCovarianceRecvClkErrUnits::s;

    /// @brief GUI selection of the initial covariance for the inter-constellation clock error
    double gui_initCovarianceInterSysErr = 1e-8;
    /// @brief Initial covariance for the inter-constellation clock error
    double _initCovarianceInterSysErr = 1e-8;

    // ###########################################################################################################

    /// Gui selection for the Unit of the initial covariance for the inter-system clock drift
    InitCovarianceRecvClkDriftUnits gui_initCovarianceInterSysDriftUnit = InitCovarianceRecvClkDriftUnits::s_s;

    /// @brief GUI selection of the initial covariance for the inter-constellation clock drift
    double gui_initCovarianceInterSysDrift = 1e-10;
    /// @brief Initial covariance for the inter-constellation clock drift
    double _initCovarianceInterSysDrift = 1e-10;

    // ###########################################################################################################

    /// GUI option for the Q calculation algorithm
    enum class QCalculationAlgorithm
    {
        VanLoan, ///< Van-Loan
        Taylor1, ///< Taylor
    };
    /// Q calculation algorithm
    QCalculationAlgorithm qCalculationAlgorithm = QCalculationAlgorithm::VanLoan;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the acceleration due to user motion
    enum class CovarianceAccelUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input stdev_accel parameter for the StDev due to acceleration due to user motion
    CovarianceAccelUnits gui_covarianceAccelUnit = CovarianceAccelUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    Eigen::Vector2d gui_covarianceAccel = { 3.0, 1.5 } /* [ m / √(s^3) ] */;
    /// @brief Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    Eigen::Vector2d _covarianceAccel = { 3.0, 1.5 } /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock phase drift
    enum class CovarianceClkPhaseDriftUnits
    {
        m_sqrts3, ///< [ m / √(s^3) ]
        m2_s3,    ///< [ m^2 / s^3 ]
    };
    /// Gui selection for the Unit of the input stdevClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits gui_covarianceClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the clock phase drift
    double gui_covarianceClkPhaseDrift = 0.2 /* [ m / √(s^3) ] */;
    /// @brief Covariance of the clock phase drift
    double _covarianceClkPhaseDrift = 0.2 /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Possible Units for the Standard deviation of the clock frequency drift
    enum class CovarianceClkFrequencyDriftUnits
    {
        m_sqrts, ///< [ m / √(s) ]
        m2_s,    ///< [ m^2 / s ]
    };
    /// Gui selection for the Unit of the input stdevClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits gui_covarianceClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m_sqrts;

    /// @brief GUI selection for the Standard deviation of the clock frequency drift
    double gui_covarianceClkFrequencyDrift = 0.1 /* [ m / √(s) ] */;
    /// @brief Covariance of the clock frequency drift
    double _covarianceClkFrequencyDrift = 0.1 /* [ m^2 / s ] */;

    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdevClkPhaseDrift parameter
    CovarianceClkPhaseDriftUnits gui_covarianceInterSysClkPhaseDriftUnit = CovarianceClkPhaseDriftUnits::m_sqrts3;

    /// @brief GUI selection for the Standard deviation of the inter-system clock phase drift
    double gui_covarianceInterSysClkPhaseDrift = std::sqrt(2) * 0.2 /* [ m / √(s^3) ] */;
    /// @brief Covariance of the inter-system clock phase drift
    double _covarianceInterSysClkPhaseDrift = std::sqrt(2) * 0.2 /* [ m^2 / s^3 ] */;

    // ###########################################################################################################

    /// Gui selection for the Unit of the input stdevClkFrequencyDrift parameter
    CovarianceClkFrequencyDriftUnits gui_covarianceInterSysClkFrequencyDriftUnit = CovarianceClkFrequencyDriftUnits::m_sqrts;

    /// @brief GUI selection for the Standard deviation of the inter-system clock frequency drift
    double gui_covarianceInterSysClkFrequencyDrift = std::sqrt(2) * 0.1 /* [ m / √(s) ] */;
    /// @brief Covariance of the inter-system clock frequency drift
    double _covarianceInterSysClkFrequencyDrift = std::sqrt(2) * 0.1 /* [ m^2 / s ] */;

    // ###########################################################################################################

    /// @brief Creates the GUI for SPP if a Kalman Filter is selected
    void gui();

    /// @brief Initializes and resets the filter
    void initialize();

    /// @brief Checks wether the Kalman filter is initialized
    bool isInitialized() const;

    /// @brief Sets the vector that contains all satellite systems that the user has selected in the GUI besides Reference time satellite system
    /// @param[in] allSatSys selected satellite systems besides Reference time satellite system
    void setAllSatSysExceptRef(const std::vector<SatelliteSystem>& allSatSys);

    /// @ brief Gets the vector that contains all satellite systems that the user has selected in the GUI besides Reference time satellite system
    std::vector<SatelliteSystem> getAllSatSysExceptRef() const;

    /// @ brief Gets satellite system used as time reference
    SatelliteSystem getRefTimeSatSys() const;

    /// @brief Gets the state keys for inter-system clock estimation
    /// @param[in] insTime Epoch time
    void addInterSysStateKeys(const InsTime& insTime);

    /// @brief Initialize Kalman Filter from LSE solution
    /// @param[in] sppSolLSE SPP Least squares solution
    /// @param[in] interSysErrs Inter-system clock error keys
    /// @param[in] interSysDrifts Inter-system clock drift keys
    void initializeKalmanFilter(std::shared_ptr<SppSolution>& sppSolLSE,
                                const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysErrs,
                                const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysDrifts);

    /// @brief Performs Single Point Positioning algorithm based on Kalman Filter
    /// @param[in] insTime Epoch time
    /// @param[in, out] calcData Data calculated for each satellite, that is available and selected
    /// @param[in] ionosphericCorrections Collection of all connected Ionospheric Corrections
    /// @param[in] ionosphereModel Ionosphere Model used for the calculation
    /// @param[in] troposphereModels Troposphere Models used for the calculation
    /// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
    /// @param[in] elevationMask Elevation cut-off angle for satellites in [rad]
    /// @param[in] snrMask Signal-to-Noise ratio mask
    /// @param[in] useDoppler Boolean which enables the use of doppler observations
    /// @param[in, out] interSysErrs Inter-system clock error keys
    /// @param[in, out] interSysDrifts Inter-system clock drift keys
    /// @return Shared pointer to the SPP solution
    std::shared_ptr<NAV::SppSolution> estimateSppSolution(const InsTime& insTime,
                                                          std::vector<CalcData>& calcData,
                                                          const IonosphericCorrections& ionosphericCorrections,
                                                          const IonosphereModel& ionosphereModel,
                                                          const TroposphereModelSelection& troposphereModels,
                                                          const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                          double elevationMask,
                                                          const SNRMask& snrMask,
                                                          bool useDoppler,
                                                          std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysErrs,
                                                          std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysDrifts);

  private:
    /// @brief Does the Kalman Filter prediction
    /// @param[in] insTime Epoch time
    void kalmanFilterPrediction(const InsTime& insTime);

    /// @brief Does the Kalman Filter update
    /// @param[in] dpsr Difference between Pseudorange measurements and estimates [m]
    /// @param[in] e_H_psr Measurement/Geometry matrix for the pseudorange
    /// @param[in] W_psr Pseudorange measurement error weight matrix
    /// @param[in] dpsr_dot Difference between Pseudorange rate measurements and estimates [m/s]
    /// @param[in] e_H_r Measurement/Geometry matrix for the pseudorange-rate
    /// @param[in] W_psrRate Pseudorange rate (doppler) measurement error weight matrix
    /// @param[in] sppSolReferenceTimeSatelliteSystem Reference time satellite system selected from SPP LSE functions
    /// @param[in] useDoppler Boolean which enables the use of doppler observations
    /// @param[in] interSysErrs Inter-system clock error keys
    /// @param[in] interSysDrifts Inter-system clock drift keys
    /// @param[in] insTime Epoch time
    void kalmanFilterUpdate(const KeyedVectorXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes>& dpsr,
                            const KeyedMatrixXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& e_H_psr,
                            const KeyedMatrixXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes, GNSS::Positioning::SPP::Meas::MeasKeyTypes>& W_psr,
                            const KeyedVectorXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes>& dpsr_dot,
                            const KeyedMatrixXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& e_H_r,
                            const KeyedMatrixXd<GNSS::Positioning::SPP::Meas::MeasKeyTypes, GNSS::Positioning::SPP::Meas::MeasKeyTypes>& W_psrRate,
                            SatelliteSystem sppSolReferenceTimeSatelliteSystem,
                            bool useDoppler,
                            const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysErrs,
                            const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysDrifts,
                            const InsTime& insTime);

    /// @brief Sets Process Noise Matrix
    /// @param[in] dt Time interval between the Kalman Filter estimates in [s]
    void processNoiseMatrixGroves(double dt);

    /// @brief Assigns Kalman Filter estimates to internal variables
    /// @param[in] otherSatelliteSystems List of satellite systems (all, or available at this epoch)
    void assignInternalSolution(const std::vector<SatelliteSystem>& otherSatelliteSystems);

    /// @brief Assigns Kalman Filter estimates to internal variables as well as sppSol and state
    /// @param[in, out] sppSol Single Point Positioning Solution
    /// @param[in] otherSatelliteSystems List of satellite systems (all, or available at this epoch)
    void assignSolution(std::shared_ptr<NAV::SppSolution>& sppSol,
                        const std::vector<SatelliteSystem>& otherSatelliteSystems);

    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d _e_position = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d _e_velocity = Eigen::Vector3d::Zero();

    /// Estimated receiver clock parameters
    ReceiverClock _recvClk;

    /// Time of last KF estimation
    InsTime _lastUpdate;

    /// Kalman Filter representation
    KeyedKalmanFilterD<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::Meas::MeasKeyTypes> _kalmanFilter;

    /// Satellite system used as time reference
    SatelliteSystem _refTimeSatSys = SatSys_None;

    /// All satellite systems that the user has selected in the GUI besides Reference time satellite system
    std::vector<SatelliteSystem> _allSatSysExceptRef;

    /// Boolean that determines, if Kalman Filter is initialized (from weighted LSE solution)
    bool _initialized = false;
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const SppKalmanFilter& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, SppKalmanFilter& data);

} // namespace NAV::GNSS::Positioning::SPP
