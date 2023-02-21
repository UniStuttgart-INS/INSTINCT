// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file StandardAtmosphere.hpp
/// @brief Standard Atmosphere pressure model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <gcem.hpp>

namespace NAV
{

/// @brief Calculates the standard atmosphere total pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @return The total pressure in [hPa] = [mbar]
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.1, p. 149
[[nodiscard]] constexpr double calcTotalPressureStAtm(double altitudeMSL)
{
    return 1013.25 * gcem::pow(1 - 2.2557e-5 * altitudeMSL, 5.2568);
}

} // namespace NAV
