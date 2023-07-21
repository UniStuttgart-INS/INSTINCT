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

namespace NAV
{
namespace GNSS::Positioning::SPP
{

/// Possible SPP estimation algorithms
enum class EstimatorType
{
    LEAST_SQUARES,          ///< Linear Least Squares
    WEIGHTED_LEAST_SQUARES, ///< Weighted Linear Least Squares
    KF,                     ///< Kalman Filter
    COUNT,                  ///< Amount of items in the enum
};

/// @brief Shows a ComboBox to select the SPP estimator
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] estimatorType Reference to the SPP estimator to select
bool ComboSppEstimatorType(const char* label, EstimatorType& estimatorType);

/// @brief State estimated by the SPP algorithm
struct State
{
    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d e_position = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d e_velocity = Eigen::Vector3d::Zero();
    /// Estimated receiver clock parameters
    ReceiverClock recvClk;
};

/// @brief Calculates the SPP solution with a Least squares estimator
/// @param[in] state Previous SPP state
/// @param[in] gnssObs GNSS observation received
/// @param[in] gnssNavInfos Collection of all connected navigation data providers
/// @param[in] ionosphereModel Ionosphere Model used for the calculation
/// @param[in] troposphereModels Troposphere Models used for the calculation
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
                                                const EstimatorType& estimatorType,
                                                const Frequency& filterFreq,
                                                const Code& filterCode,
                                                const std::vector<SatId>& excludedSatellites,
                                                double elevationMask);

} // namespace GNSS::Positioning::SPP

/// @brief Converts the enum to a string
/// @param[in] estimatorType Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GNSS::Positioning::SPP::EstimatorType estimatorType);

} // namespace NAV
