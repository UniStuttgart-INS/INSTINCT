// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GMFCoeffs.hpp
/// @brief GMF Coefficients
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-04-21
/// @note See \cite Böhm2006a Böhm2006: Global Mapping Function (GMF): A new empirical mapping function based on numerical weather model data
/// @note See https://vmf.geo.tuwien.ac.at/codes/ for code sources in matlab.

#pragma once

#include <array>

namespace NAV::internal::GMF
{

/// @brief Hydrostatic coefficient 'a' mean values
extern std::array<double, 55> ah_mean;
/// @brief Hydrostatic coefficient 'b' mean values
extern std::array<double, 55> bh_mean;
/// @brief Hydrostatic coefficient 'a' amplitude
extern std::array<double, 55> ah_amp;
/// @brief Hydrostatic coefficient 'b' amplitude
extern std::array<double, 55> bh_amp;

/// @brief Wet coefficient 'a' mean values
extern std::array<double, 55> aw_mean;
/// @brief Wet coefficient 'b' mean values
extern std::array<double, 55> bw_mean;
/// @brief Wet coefficient 'a' amplitude
extern std::array<double, 55> aw_amp;
/// @brief Wet coefficient 'b' amplitude
extern std::array<double, 55> bw_amp;

} // namespace NAV::internal::GMF
