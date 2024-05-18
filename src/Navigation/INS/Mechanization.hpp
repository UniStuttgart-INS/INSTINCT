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
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
struct PosVelAttDerivativeConstants
{
    GravitationModel gravitationModel = GravitationModel::EGM96; ///< Gravity Model to use
    bool coriolisAccelerationCompensationEnabled = true;         ///< Apply the Coriolis acceleration compensation to the measured accelerations
    bool centrifgalAccelerationCompensationEnabled = true;       ///< Apply the centrifugal acceleration compensation to the measured accelerations
    bool angularRateEarthRotationCompensationEnabled = true;     ///< Apply the Earth rotation rate compensation to the measured angular rates
    bool angularRateTransportRateCompensationEnabled = true;     ///< Apply the transport rate compensation to the measured angular rates
    Scalar timeDifferenceSec = 0.0;                              ///< Time difference over the whole integration step [s]
};

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
void to_json(json& j, const PosVelAttDerivativeConstants<Scalar>& data)
{
    j = json{
        { "gravitationModel", data.gravitationModel },
        { "coriolisAccelerationCompensationEnabled", data.coriolisAccelerationCompensationEnabled },
        { "centrifgalAccelerationCompensationEnabled", data.centrifgalAccelerationCompensationEnabled },
        { "angularRateEarthRotationCompensationEnabled", data.angularRateEarthRotationCompensationEnabled },
        { "angularRateTransportRateCompensationEnabled", data.angularRateTransportRateCompensationEnabled },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
template<typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
void from_json(const json& j, PosVelAttDerivativeConstants<Scalar>& data)
{
    if (j.contains("gravitationModel")) { j.at("gravitationModel").get_to(data.gravitationModel); }
    if (j.contains("coriolisAccelerationCompensationEnabled")) { j.at("coriolisAccelerationCompensationEnabled").get_to(data.coriolisAccelerationCompensationEnabled); }
    if (j.contains("centrifgalAccelerationCompensationEnabled")) { j.at("centrifgalAccelerationCompensationEnabled").get_to(data.centrifgalAccelerationCompensationEnabled); }
    if (j.contains("angularRateEarthRotationCompensationEnabled")) { j.at("angularRateEarthRotationCompensationEnabled").get_to(data.angularRateEarthRotationCompensationEnabled); }
    if (j.contains("angularRateTransportRateCompensationEnabled")) { j.at("angularRateTransportRateCompensationEnabled").get_to(data.angularRateTransportRateCompensationEnabled); }
}

} // namespace NAV