// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Vector.hpp
/// @brief Vector Utility functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-07

#pragma once

#include <vector>

namespace NAV
{

/// @brief Moves an element within a vector to a new position
/// @param[in, out] v Vector with the elements
/// @param[in] sourceIdx Old index which will be moved
/// @param[in] targetIdx New index which is the target
template<typename T>
void move(std::vector<T>& v, size_t sourceIdx, size_t targetIdx)
{
    if (sourceIdx > targetIdx)
    {
        std::rotate(v.rend() - static_cast<int64_t>(sourceIdx) - 1,
                    v.rend() - static_cast<int64_t>(sourceIdx), v.rend() - static_cast<int64_t>(targetIdx));
    }
    else
    {
        std::rotate(v.begin() + static_cast<int64_t>(sourceIdx),
                    v.begin() + static_cast<int64_t>(sourceIdx) + 1, v.begin() + static_cast<int64_t>(targetIdx) + 1);
    }
}

} // namespace NAV
