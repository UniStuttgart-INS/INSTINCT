// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImGui.hpp
/// @brief ImGui Helper
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-24

#pragma once

#include <imgui.h>

/// @brief Add operator
/// @param lhs Left-hand side
/// @param rhs Right-hand side
/// @return Computation result
constexpr ImVec4 operator+(const ImVec4& lhs, const ImVec4& rhs)
{
    return { lhs.x + rhs.x,
             lhs.y + rhs.y,
             lhs.z + rhs.z,
             lhs.w + rhs.w };
}

/// @brief Subtract operator
/// @param lhs Left-hand side
/// @param rhs Right-hand side
/// @return Computation result
constexpr ImVec4 operator-(const ImVec4& lhs, const ImVec4& rhs)
{
    return { lhs.x - rhs.x,
             lhs.y - rhs.y,
             lhs.z - rhs.z,
             lhs.w - rhs.w };
}

/// @brief Scalar multiplication operator
/// @param lhs Left-hand side
/// @param rhs Right-hand side
/// @return Computation result
constexpr ImVec4 operator*(const float& lhs, const ImVec4& rhs)
{
    return { lhs * rhs.x,
             lhs * rhs.y,
             lhs * rhs.z,
             lhs * rhs.w };
}

/// @brief Scalar multiplication operator
/// @param lhs Left-hand side
/// @param rhs Right-hand side
/// @return Computation result
constexpr ImVec4 operator*(const ImVec4& lhs, const float& rhs)
{
    return { lhs.x * rhs,
             lhs.y * rhs,
             lhs.z * rhs,
             lhs.w * rhs };
}

/// @brief Scalar multiplication operator
/// @param lhs Left-hand side
/// @param rhs Right-hand side
/// @return Computation result
constexpr ImVec4 operator/(const ImVec4& lhs, const float& rhs)
{
    return { lhs.x / rhs,
             lhs.y / rhs,
             lhs.z / rhs,
             lhs.w / rhs };
}