// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file StandardAtmosphere.hpp
/// @brief Standard Atmosphere temperature model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

namespace NAV
{

/// @brief Calculates the standard atmosphere absolute temperature
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level)
/// @return The absolute temperature in [K]
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.2, p. 149
[[nodiscard]] constexpr double calcAbsoluteTemperatureStAtm(double altitudeMSL)
{
    return 15.0 - 6.5e-3 * altitudeMSL + 273.15;
}

} // namespace NAV
