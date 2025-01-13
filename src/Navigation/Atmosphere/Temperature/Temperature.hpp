// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Temperature.hpp
/// @brief Temperature calculation formulas
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <cstdint>
#include <fmt/format.h>

namespace NAV
{

/// Available temperature Models
enum class TemperatureModel : uint8_t
{
    None,    ///< No temperature model
    ConstNN, ///< Constant value at zero altitude
    ISA,     ///< ICAO Standard Atmosphere
    GPT2,    ///< GPT2
    GPT3,    ///< GPT3
    COUNT,   ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] temperatureModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(TemperatureModel temperatureModel);

/// @brief Shows a ComboBox to select the temperature model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] temperatureModel Reference to the temperature model to select
bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel);

/// @brief Calculates the absolute temperature
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @param[in] temperatureModel Temperature model to use
/// @return The absolute temperature in [K]
[[nodiscard]] double calcAbsoluteTemperature(double altitudeMSL, TemperatureModel temperatureModel);

} // namespace NAV

/// @brief Formatter
template<>
struct fmt::formatter<NAV::TemperatureModel> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] data Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::TemperatureModel& data, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(NAV::to_string(data), ctx);
    }
};