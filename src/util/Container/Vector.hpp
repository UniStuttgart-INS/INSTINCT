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
