// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ZenithDelay.hpp
/// @brief Zenith hydrostatic and wet delay
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

namespace NAV
{

/// Zenith delays and mapping factors
struct ZenithDelay
{
    double ZHD{};              ///< Zenith hydrostatic delay [m]
    double ZWD{};              ///< Zenith wet delay [m]
    double zhdMappingFactor{}; ///< Zenith hydrostatic delay mapping factor
    double zwdMappingFactor{}; ///< Zenith wet delay mapping factor
};

} // namespace NAV
