// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Pair.hpp
/// @brief Utility functions for std::pair
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-19

#pragma once

#include <cstddef>
#include <utility>
#include <functional>

namespace std
{
/// @brief A hash function used to hash a pair of any kind (needed for unordered_map)
template<class T1, class T2>
struct hash<std::pair<T1, T2>>
{
    /// @brief Hash function for std::pair
    /// @param[in] f Pair
    std::size_t operator()(const std::pair<T1, T2>& f) const
    {
        auto hash1 = std::hash<T1>{}(f.first);
        auto hash2 = std::hash<T2>{}(f.second);

        if (hash1 != hash2)
        {
            return hash1 ^ hash2 << 1;
        }

        // If hash1 == hash2, their XOR is zero.
        return hash1;
    }
};
} // namespace std