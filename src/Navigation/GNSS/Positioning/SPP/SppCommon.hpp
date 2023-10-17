// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppCommon.hpp
/// @brief Common Functions for the SPP algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-21

#pragma once

#include <vector>
#include <set>
#include <unordered_map>

#include "util/Eigen.hpp"
#include "util/Container/KeyedMatrix.hpp"

#include "Navigation/GNSS/Positioning/SppAlgorithmTypes.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/GNSS/Errors.hpp"
#include "Navigation/GNSS/Positioning/SPP/SppKeys.hpp"

namespace States = NAV::GNSS::Positioning::SPP::States;
namespace Meas = NAV::GNSS::Positioning::SPP::Meas;

namespace NAV::GNSS::Positioning::SPP
{

/// Data calculated for each observation
struct CalcData
{
    /// Constructor
    /// @param[in] obsData Observation data
    /// @param[in] satNavData Satellite Navigation data
    explicit CalcData(const NAV::GnssObs::ObservationData& obsData, std::shared_ptr<NAV::SatNavData> satNavData)
        : obsData(obsData), satNavData(std::move(satNavData)) {}

    const NAV::GnssObs::ObservationData& obsData;          ///< GNSS Observation data
    std::shared_ptr<NAV::SatNavData> satNavData = nullptr; ///< Satellite Navigation data

    double satClkBias{};                       ///< Satellite clock bias [s]
    double satClkDrift{};                      ///< Satellite clock drift [s/s]
    Eigen::Vector3d e_satPos;                  ///< Satellite position in ECEF frame coordinates [m]
    Eigen::Vector3d e_satVel;                  ///< Satellite velocity in ECEF frame coordinates [m/s]
    double pseudorangeEst = 0.0;               ///< Estimated Pseudorange [m]
    std::optional<double> pseudorangeRateMeas; ///< Measured Pseudorange rate [m/s]

    // Data recalculated each iteration

    bool skipped = false;                    ///< Whether to skip the measurement
    Eigen::Vector3d e_lineOfSightUnitVector; ///< Line-of-sight unit vector in ECEF frame coordinates
    Eigen::Vector3d n_lineOfSightUnitVector; ///< Line-of-sight unit vector in NED frame coordinates
    double satElevation = 0.0;               ///< Elevation [rad]
    double satAzimuth = 0.0;                 ///< Azimuth [rad]

    int8_t freqNum = -128; ///< Frequency number (GLONASS only)
};

/// @brief Selects the satellite signals depending on various filters
/// @param[in] gnssObs GNSS observations (pseudoranges, carrier phases, doppler, CN0, ...)
/// @param[in] gnssNavInfos List of Navigation data (ionospheric corrections, time system corrections, ...)
/// @param[in] filterFreq Frequencies to use
/// @param[in] filterCode Codes to use
/// @param[in] excludedSatellites List of excluded satellites
/// @return Data calculated for each satellite, that is available and selected
std::vector<CalcData> selectObservations(const std::shared_ptr<const GnssObs>& gnssObs,
                                         const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                         const Frequency& filterFreq,
                                         const Code& filterCode,
                                         const std::vector<SatId>& excludedSatellites);

/// @brief Find all observations providing a doppler measurement
/// @param[out] calcData Data calculated for each satellite, that is available and selected
/// @return Number of Doppler measurements
size_t findDopplerMeasurements(std::vector<CalcData>& calcData);

/// @brief Struct containing a value and its weight for estimation
/// @tparam T Scalar type
template<typename T>
struct ValueWeight
{
    T value;  ///< Value
    T weight; ///< Weight for estimation
};

/// @brief Calculate pseudorange estimate and weight
/// @param[out] sppSol SPP solution
/// @param[in] calc Data calculated for each satellite, that is available and selected
/// @param[in] insTime Epoch time
/// @param[in] state SPP state from the previous epoch
/// @param[in] lla_pos Position in latitude, longitude, altitude in [rad, rad, m]
/// @param[in] ionosphericCorrections Ionospheric corrections from the navigation data
/// @param[in] ionosphereModel Ionospheric model to use
/// @param[in] troposphereModels Troposphere mode to use
/// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
/// @param[in] estimatorType Estimator type
/// @return Pseudorange and its weight
ValueWeight<double> calcPsrAndWeight(const std::shared_ptr<SppSolution>& sppSol,
                                     const CalcData& calc,
                                     const InsTime& insTime,
                                     State& state,
                                     const Eigen::Vector3d& lla_pos,
                                     const IonosphericCorrections& ionosphericCorrections,
                                     const IonosphereModel& ionosphereModel,
                                     const TroposphereModelSelection& troposphereModels,
                                     const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                     const EstimatorType& estimatorType);

/// @brief Calculate pseudorange rate estimate and weight
/// @param[in] calc Data calculated for each satellite, that is available and selected
/// @param[in] state SPP state from the previous epoch
/// @param[in] estimatorType Estimator type
/// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
/// @return Pseudorange rate and its weight
ValueWeight<double> calcPsrRateAndWeight(const CalcData& calc,
                                         State& state,
                                         const EstimatorType& estimatorType,
                                         const GnssMeasurementErrorModel& gnssMeasurementErrorModel);

/// @brief Return type for the function 'calcMeasurementEstimatesAndDesignMatrix'
struct EstWeightDesignMatrices
{
    /// e_H_psr Measurement/Geometry matrix for the pseudorange
    KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes> e_H_psr;
    /// psrEst Pseudorange estimates [m]
    KeyedVectorXd<Meas::MeasKeyTypes> psrEst;
    /// psrMeas Pseudorange measurements [m]
    KeyedVectorXd<Meas::MeasKeyTypes> psrMeas;
    /// W_psr Pseudorange measurement error weight matrix
    KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes> W_psr;
    /// dpsr Difference between Pseudorange measurements and estimates
    KeyedVectorXd<Meas::MeasKeyTypes> dpsr;

    /// e_H_r Measurement/Geometry matrix for the pseudorange-rate
    KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes> e_H_r;
    /// psrRateEst Corrected pseudorange-rate estimates [m/s]
    KeyedVectorXd<Meas::MeasKeyTypes> psrRateEst;
    /// psrRateMeas Corrected pseudorange-rate measurements [m/s]
    KeyedVectorXd<Meas::MeasKeyTypes> psrRateMeas;
    /// W_psrRate Pseudorange rate (doppler) measurement error weight matrix
    KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes> W_psrRate;
    /// dpsr_dot Difference between Pseudorange rate measurements and estimates
    KeyedVectorXd<Meas::MeasKeyTypes> dpsr_dot;
};

/// @brief Get Measurements, it's estimates and the corresponding Design-Matrix
/// @param[out] sppSol SPP solution
/// @param[in, out] calcData Data calculated for each satellite, that is available and selected
/// @param[in] insTime Epoch time
/// @param[in] state SPP state from the previous epoch
/// @param[in] lla_pos Position in latitude, longitude, altitude in [rad, rad, m]
/// @param[in] ionosphericCorrections Ionospheric corrections from the navigation data
/// @param[in] ionosphereModel Ionospheric model to use
/// @param[in] troposphereModels Troposphere mode to use
/// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
/// @param[in] estimatorType Estimator type
/// @param[in] useDoppler Boolean which enables the use of doppler observations
/// @param[in] interSysErrs Inter-system clock error keys
/// @param[in] interSysDrifts Inter-system clock drift keys
/// @return Matrices: e_H_psr, psrEst, psrMeas, W_psr, dpsr, e_H_r, psrRateEst, psrRateMeas, W_psrRate, dpsr_dot
EstWeightDesignMatrices calcMeasurementEstimatesAndDesignMatrix(const std::shared_ptr<SppSolution>& sppSol,
                                                                std::vector<CalcData>& calcData,
                                                                const InsTime& insTime,
                                                                State& state,
                                                                const Eigen::Vector3d& lla_pos,
                                                                const IonosphericCorrections& ionosphericCorrections,
                                                                const IonosphereModel& ionosphereModel,
                                                                const TroposphereModelSelection& troposphereModels,
                                                                const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                                const EstimatorType& estimatorType,
                                                                bool useDoppler,
                                                                const std::vector<States::StateKeyTypes>& interSysErrs,
                                                                const std::vector<States::StateKeyTypes>& interSysDrifts);

/// @brief Prepares data further based on the current estimation and applies elevation mask
/// @param[out] sppSol SPP solution
/// @param[in, out] satelliteSystems List of satellite systems of this epoch (systems can be removed if all satellited are skipped due to e.g. elevation mask)
/// @param[in, out] calcData Data calculated for each satellite, that is available and selected
/// @param[in, out] state SPP state from the previous epoch (reference time satellite system gets updated to the new epoch)
/// @param[in] nParam Number of parameters
/// @param[in] nMeasPsr Number of pseudorange measurements
/// @param[in] nMeasDoppler Number of doppler/pseudorange rate measurements
/// @param[in] insTime Epoch time
/// @param[in] lla_pos Position in latitude, longitude, altitude in [rad, rad, m]
/// @param[in] elevationMask Elevation cut-off angle for satellites in [rad]
/// @param[in] estimatorType Estimator type
/// @return False if no calculation is possible
bool calcDataBasedOnEstimates(const std::shared_ptr<SppSolution>& sppSol,
                              std::vector<SatelliteSystem>& satelliteSystems,
                              std::vector<CalcData>& calcData,
                              State& state,
                              size_t nParam,
                              size_t nMeasPsr,
                              size_t nMeasDoppler,
                              const InsTime& insTime,
                              const Eigen::Vector3d& lla_pos,
                              double elevationMask,
                              EstimatorType estimatorType);

/// @brief Gets keys for inter-system clock errors and drifts
/// @param[in] satelliteSystems List of satellite systems of current epoch
/// @param[in, out] interSysErrs Inter-system clock error keys
/// @param[in, out] interSysDrifts Inter-system clock drift keys
void getInterSysKeys(const std::vector<SatelliteSystem>& satelliteSystems,
                     std::vector<States::StateKeyTypes>& interSysErrs,
                     std::vector<States::StateKeyTypes>& interSysDrifts);

} // namespace NAV::GNSS::Positioning::SPP