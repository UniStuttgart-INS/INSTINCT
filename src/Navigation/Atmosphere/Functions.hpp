// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Functions.hpp
/// @brief Atmosphere helper functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

namespace NAV
{

/// @brief Calculates the standard atmosphere total pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level)
/// @return The total pressure in [hPa] = [mbar]
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.1, p. 149
[[nodiscard]] double calcTotalPressure(double altitudeMSL);

/// @brief Calculates the standard atmosphere absolute temperature
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level)
/// @return The absolute temperature in [K]
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.2, p. 149
[[nodiscard]] double calcAbsoluteTemperature(double altitudeMSL);

/// @brief Calculates the standard atmosphere partial pressure of water vapor
/// @param[in] temp The absolute temperature in [K]
/// @param[in] humidity_rel The relative humidity
/// @return The partial pressure [hPa] of water vapor
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.3, p. 149
[[nodiscard]] double calcWaterVaporPartialPressure(double temp, double humidity_rel);

} // namespace NAV
