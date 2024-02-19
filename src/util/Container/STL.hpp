// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file STL.hpp
/// @brief Algorithms concerning the STL containers
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-11

#pragma once

#include <fmt/format.h>
#include <cstring>
#include <string>
#include <functional>

namespace NAV
{

/// @brief Joins the container to a string
/// @param[in] container Container to join
/// @param[in] delimiter Delimiter to use to concatenate the elements
/// @param[in] elementFormat fmt format string, which gets places inside '{}' to format the elements
template<typename T>
std::string joinToString(const T& container, const char* delimiter = ", ", const std::string& elementFormat = "")
{
    std::string text;
    std::for_each(container.begin(), container.end(), [&](const auto& element) {
        text += fmt::format(fmt::runtime("{" + elementFormat + "}{}"), element, delimiter);
    });
    return text.substr(0, text.length() - strlen(delimiter));
}

/// @brief Joins the container to a string
/// @param[in] container Container to join
/// @param[in] formatFunc Function to evaluate for each element to get the string representation
/// @param[in] delimiter Delimiter to use to concatenate the elements
template<typename T, typename U>
std::string joinToStringCustom(const T& container, U&& formatFunc, const char* delimiter = ", ")
{
    std::string text;
    std::for_each(container.begin(), container.end(), [&](const auto& element) {
        text += fmt::format("{}{}", formatFunc(element), delimiter);
    });
    return text.substr(0, text.length() - strlen(delimiter));
}

} // namespace NAV
