// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GMF.hpp
/// @brief Global Mapping Function (GMF)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-04-21
/// @note See \cite Böhm2006a Böhm2006: Global Mapping Function (GMF): A new empirical mapping function based on numerical weather model data
/// @note See https://vmf.geo.tuwien.ac.at/codes/ for code sources in matlab.

#pragma once

#include <Eigen/Core>

namespace NAV
{

/// @brief Calculates the Global Mapping Function (GMF) for the hydrostatic delay
/// @param[in] mjd Modified julian date
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @return Hydrostatic mapping function
double calcTropoMapFunc_GMFH(double mjd, const Eigen::Vector3d& lla_pos, double elevation);

/// @brief Calculates the Global Mapping Function (GMF) for the wet delay
/// @param[in] mjd Modified julian date
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @return Wet mapping function
double calcTropoMapFunc_GMFW(double mjd, const Eigen::Vector3d& lla_pos, double elevation);

} // namespace NAV