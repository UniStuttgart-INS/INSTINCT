// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SinglePointPositioning.hpp
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

/// Possible SPP estimation algorithms
enum class SppEstimator
{
    LEAST_SQUARES,          ///< Linear Least Squares
    WEIGHTED_LEAST_SQUARES, ///< Weighted Linear Least Squares
    COUNT,                  ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] sppEstimator Enum value to convert into text
/// @return String representation of the enum
const char* to_string(SppEstimator sppEstimator);

/// @brief Shows a ComboBox to select the SPP estimator
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] sppEstimator Reference to the SPP estimator to select
bool ComboSppEstimator(const char* label, SppEstimator& sppEstimator);

/// @brief State estimated by the SPP algorithm
struct SppState
{
    /// Estimated position in ECEF frame [m]
    Eigen::Vector3d e_position = Eigen::Vector3d::Zero();
    /// Estimated velocity in ECEF frame [m/s]
    Eigen::Vector3d e_velocity = Eigen::Vector3d::Zero();
    /// Estimated receiver clock parameters
    ReceiverClock recvClk;
};

/// @brief Calculates the SPP solution
/// @param state Previous SPP state
/// @param gnssObs GNSS observation received
/// @param gnssNavInfos Collection of all connected navigation data providers
/// @param ionosphereModel Ionosphere Model used for the calculation
/// @param troposphereModels Troposphere Models used for the calculation
/// @param sppEstimator Estimation algorithm used
/// @param filterFreq Frequencies used for calculation (GUI filter)
/// @param filterCode Codes used for calculation (GUI filter)
/// @param excludedSatellites List of satellites to exclude
/// @param elevationMask Elevation cut-off angle for satellites in [rad]
/// @return Shared pointer to the SPP solution
std::shared_ptr<const SppSolution> calcSppSolution(SppState state,
                                                   const std::shared_ptr<const GnssObs>& gnssObs,
                                                   const std::vector<const GnssNavInfo*>& gnssNavInfos,
                                                   const IonosphereModel& ionosphereModel,
                                                   const TroposphereModelSelection& troposphereModels,
                                                   const SppEstimator& sppEstimator,
                                                   const Frequency& filterFreq,
                                                   const Code& filterCode,
                                                   const std::vector<SatId>& excludedSatellites,
                                                   double elevationMask);

} // namespace NAV
