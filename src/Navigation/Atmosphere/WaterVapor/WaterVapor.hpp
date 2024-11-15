// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WaterVapor.hpp
/// @brief Water vapor calculation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <cstdint>

namespace NAV
{

/// Available Water vapor Models
enum class WaterVaporModel : uint8_t
{
    None,  ///< Water vapor model turned off
    ISA,   ///< ICAO Standard Atmosphere
    GPT2,  ///< GPT2
    GPT3,  ///< GPT3
    COUNT, ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] waterVaporModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(WaterVaporModel waterVaporModel);

/// @brief Shows a ComboBox to select the water vapor model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] waterVaporModel Reference to the water vapor model to select
bool ComboWaterVaporModel(const char* label, WaterVaporModel& waterVaporModel);

/// @brief Calculates the partial pressure of water vapor
/// @param[in] temp The absolute temperature in [K]
/// @param[in] humidity_rel The relative humidity
/// @param[in] waterVaporModel Water vapor model to use
/// @return The partial pressure [hPa] of water vapor
[[nodiscard]] double calcWaterVaporPartialPressure(double temp, double humidity_rel, WaterVaporModel waterVaporModel);

} // namespace NAV
