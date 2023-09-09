// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppAlgorithm.hpp
/// @brief Single Point Positioning (SPP) / Code Phase Positioning
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-05-01

#pragma once

#include <memory>
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

#include "Navigation/Atmosphere/Ionosphere/Ionosphere.hpp"
#include "Navigation/Atmosphere/Troposphere/Troposphere.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SppAlgorithmTypes.hpp"

#include "Navigation/GNSS/Positioning/SPP/SppKalmanFilter.hpp"
#include "Navigation/GNSS/Errors.hpp"

namespace NAV::GNSS::Positioning::SPP
{

/// @brief Calculates the SPP solution with a Least squares estimator
/// @param[in] state Previous SPP state
/// @param[in] gnssObs GNSS observation received
/// @param[in] gnssNavInfos Collection of all connected navigation data providers
/// @param[in] ionosphereModel Ionosphere Model used for the calculation
/// @param[in] troposphereModels Troposphere Models used for the calculation
/// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
/// @param[in] estimatorType Estimation algorithm used
/// @param[in] filterFreq Frequencies used for calculation (GUI filter)
/// @param[in] filterCode Codes used for calculation (GUI filter)
/// @param[in] excludedSatellites List of satellites to exclude
/// @param[in] elevationMask Elevation cut-off angle for satellites in [rad]
/// @return Shared pointer to the SPP solution
std::shared_ptr<SppSolution> calcSppSolutionLSE(State state,
                                                const std::shared_ptr<const GnssObs>& gnssObs,
                                                const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                const IonosphereModel& ionosphereModel,
                                                const TroposphereModelSelection& troposphereModels,
                                                const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                                const EstimatorType& estimatorType,
                                                const Frequency& filterFreq,
                                                const Code& filterCode,
                                                const std::vector<SatId>& excludedSatellites,
                                                double elevationMask);

/// @brief Calculates the SPP solution with a Kalman Filter
/// @param[in, out] kalmanFilter Spp Kalman Filter with all settings
/// @param[in] state Previous SPP state
/// @param[in] gnssObs GNSS observation received
/// @param[in] gnssNavInfos Collection of all connected navigation data providers
/// @param[in] ionosphereModel Ionosphere Model used for the calculation
/// @param[in] troposphereModels Troposphere Models used for the calculation
/// @param[in] gnssMeasurementErrorModel GNSS measurement error model to use
/// @param[in] filterFreq Frequencies used for calculation (GUI filter)
/// @param[in] filterCode Codes used for calculation (GUI filter)
/// @param[in] excludedSatellites List of satellites to exclude
/// @param[in] elevationMask Elevation cut-off angle for satellites in [rad]
/// @param[in] usedObservations Utilized observations (Order from GnssObs::ObservationType: Psr, Carrier, Doppler)
/// @return Shared pointer to the SPP solution
std::shared_ptr<SppSolution> calcSppSolutionKF(SppKalmanFilter& kalmanFilter,
                                               State state,
                                               const std::shared_ptr<const GnssObs>& gnssObs,
                                               const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                               const IonosphereModel& ionosphereModel,
                                               const TroposphereModelSelection& troposphereModels,
                                               const GnssMeasurementErrorModel& gnssMeasurementErrorModel,
                                               const Frequency& filterFreq,
                                               const Code& filterCode,
                                               const std::vector<SatId>& excludedSatellites,
                                               double elevationMask,
                                               const std::array<bool, 2>& usedObservations);

} // namespace NAV::GNSS::Positioning::SPP
