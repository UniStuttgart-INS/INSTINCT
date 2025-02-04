// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SystemModel.hpp
/// @brief System Model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-20

#pragma once

#include <cstdint>

namespace NAV
{

/// Algorithms to calculate the system model with
enum class SystemModelCalcAlgorithm : uint8_t
{
    VanLoan, ///< Van-Loan
    Taylor1, ///< Taylor
};

/// @brief Shows a GUI
/// @param[in] algorithm Algorithm to calculate the system model with
/// @param[in] itemWidth Width of the displayed items
/// @param[in] id Unique id for ImGui
/// @return True if something was changed
bool SystemModelGui(SystemModelCalcAlgorithm& algorithm, float itemWidth, const char* id);

} // namespace NAV