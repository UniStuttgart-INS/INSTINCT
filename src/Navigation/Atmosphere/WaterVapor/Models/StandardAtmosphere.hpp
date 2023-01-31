// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file StandardAtmosphere.hpp
/// @brief Standard Atmosphere water vapor model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <cmath>

namespace NAV
{

/// @brief Calculates the standard atmosphere partial pressure of water vapor
/// @param[in] temp The absolute temperature in [K]
/// @param[in] humidity_rel The relative humidity
/// @return The partial pressure [hPa] of water vapor
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.3, p. 149
[[nodiscard]] constexpr double calcWaterVaporPartialPressureStAtm(double temp, double humidity_rel)
{
    return 6.108 * std::exp((17.15 * temp - 4684.0) / (temp - 38.45)) * humidity_rel;
}

} // namespace NAV
