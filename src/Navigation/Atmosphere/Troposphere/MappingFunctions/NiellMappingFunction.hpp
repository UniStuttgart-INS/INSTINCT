// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NiellMappingFunction.hpp
/// @brief Niell Mapping Function
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-17
/// @note See https://gssc.esa.int/navipedia/index.php/Mapping_of_Niell
/// @note See \cite Niell1996 Niell1996

#pragma once

#include <Eigen/Core>
#include "Navigation/Time/InsTime.hpp"

namespace NAV
{

/// @brief Calculates the Niell Mapping Function (NMF) for the hydrostatic delay
/// @param[in] epoch Time
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @return Hydrostatic mapping function
double calcTropoMapFunc_NMFH(const InsTime& epoch, const Eigen::Vector3d& lla_pos, double elevation);

/// @brief Calculates the Niell Mapping Function (NMF) for the wet delay
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @return Wet mapping function
double calcTropoMapFunc_NMFW(const Eigen::Vector3d& lla_pos, double elevation);

} // namespace NAV