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

namespace NAV
{

/// @brief Joins the container to a string
/// @param[in] container Container to join
template<typename T>
std::string joinToString(const T& container)
{
    std::string text;
    std::for_each(container.begin(), container.end(), [&text](const auto& element) { text += fmt::format("{}, ", element); });
    return text.substr(0, text.length() - 2);
}

} // namespace NAV
