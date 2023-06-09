// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Errors.hpp
/// @brief Errors concerning GNSS observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-06-09

#pragma once

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{
/// @brief Calculates the measurement Error Variance for pseudorange observations
/// @param[in] satSys Satellite System
/// @param[in] freq Frequency
/// @param[in] elevation Satellite Elevation in [rad]
/// @return Variance of the measurement error [m^2]
[[nodiscard]] double psrMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation);

/// @brief Calculates the measurement Error Variance for carrier-phase observations
/// @param[in] satSys Satellite System
/// @param[in] freq Frequency
/// @param[in] elevation Satellite Elevation in [rad]
/// @return Variance of the measurement error [m^2]
[[nodiscard]] double carrierMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation);

/// @brief Returns the Code Bias Error Variance
/// @return Variance of the code bias error [m^2]
[[nodiscard]] double codeBiasErrorVar();

/// @brief Returns the Doppler Error Variance
/// @return Variance of the Doppler error [m^2]
[[nodiscard]] double dopplerErrorVar();

} // namespace NAV
