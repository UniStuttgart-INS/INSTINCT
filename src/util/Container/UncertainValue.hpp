// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UncertainValue.hpp
/// @brief Values with an uncertainty (Standard Deviation)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-04-19

#pragma once

namespace NAV
{

/// @brief Value with standard deviation
/// @tparam T Type of the value and standard deviation
template<typename T>
struct UncertainValue
{
    T value;  ///< Value
    T stdDev; ///< Standard deviation
};

} // namespace NAV
