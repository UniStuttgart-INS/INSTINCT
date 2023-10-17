// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppAlgorithmTypes.hpp
/// @brief Single Point Positioning (SPP) / Code Phase Positioning
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-15

#pragma once

#include "Navigation/GNSS/Core/ReceiverClock.hpp"

#include "util/Eigen.hpp"

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

} // namespace GNSS::Positioning::SPP

/// @brief Converts the enum to a string
/// @param[in] estimatorType Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GNSS::Positioning::SPP::EstimatorType estimatorType);

} // namespace NAV
