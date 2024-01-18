// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Array.hpp
/// @brief Array Utility functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-15

#pragma once

#include <array>

#include "util/Assert.h"

namespace NAV
{

namespace detail
{

/// @brief Create an array with the value assigned to all elements in the container.
/// @param[in] value Value to fill with
template<typename T, std::size_t... Is>
constexpr std::array<T, sizeof...(Is)> create_array(const T& value, std::index_sequence<Is...> /* unused */)
{
    // cast Is to void to remove the warning: unused value
    return { { (static_cast<void>(Is), value)... } };
}

} // namespace detail

/// @brief Create an array with the value assigned to all elements in the container.
/// @param[in] value Value to fill with
/// @note See https://stackoverflow.com/a/57757301
template<std::size_t N, typename T>
constexpr std::array<T, N> create_array(const T& value)
{
    return detail::create_array(value, std::make_index_sequence<N>());
}

/// @brief Returns a container filled with the given range
/// @param start Inclusive start value of the range
/// @param stepSize Step size of the range
/// @param end Exclusive end value of the range
template<size_t N, typename Scalar>
constexpr std::array<Scalar, N> genRangeArray(Scalar start, Scalar stepSize, [[maybe_unused]] Scalar end)
{
    std::array<Scalar, N> container{};

    for (size_t i = 0; i < container.size(); i++)
    {
        container.at(i) = start;
        INS_ASSERT_USER_ERROR(start < end, "The range exceeds the end value.");
        start += stepSize;
    }

    INS_ASSERT_USER_ERROR(start + stepSize > end, "The end value exceeds the range.");

    return container;
};

} // namespace NAV
