// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Mechanization.hpp
/// @brief Inertial Navigation Mechanization Functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-12-11

#pragma once

#include <type_traits>

#include "Navigation/Gravity/Gravity.hpp"

namespace NAV
{

/// @brief Values needed to calculate the PosVelAttDerivative for the local-navigation frame
struct PosVelAttDerivativeConstants
{
    GravitationModel gravitationModel = GravitationModel::EGM96; ///< Gravity Model to use
    bool coriolisAccelerationCompensationEnabled = true;         ///< Apply the Coriolis acceleration compensation to the measured accelerations
    bool centrifgalAccelerationCompensationEnabled = true;       ///< Apply the centrifugal acceleration compensation to the measured accelerations
    bool angularRateEarthRotationCompensationEnabled = true;     ///< Apply the Earth rotation rate compensation to the measured angular rates
    bool angularRateTransportRateCompensationEnabled = true;     ///< Apply the transport rate compensation to the measured angular rates
};

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const PosVelAttDerivativeConstants& data);

/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, PosVelAttDerivativeConstants& data);

} // namespace NAV