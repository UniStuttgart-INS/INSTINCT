// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Pressure.hpp
/// @brief Pressure calculation formulas
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <cstdint>
#include <fmt/format.h>

namespace NAV
{

/// Available pressure Models
enum class PressureModel : uint8_t
{
    None,    ///< No pressure model
    ConstNN, ///< Constant value at zero altitude
    ISA,     ///< ICAO Standard Atmosphere
    GPT2,    ///< GPT2
    GPT3,    ///< GPT3
    COUNT,   ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] pressureModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(PressureModel pressureModel);

/// @brief Shows a ComboBox to select the pressure model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] pressureModel Reference to the pressure model to select
bool ComboPressureModel(const char* label, PressureModel& pressureModel);

/// @brief Calculates the total pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @param[in] pressureModel Pressure model to use
/// @return The total pressure in [hPa] = [mbar]
[[nodiscard]] double calcTotalPressure(double altitudeMSL, PressureModel pressureModel);

} // namespace NAV

/// @brief Formatter
template<>
struct fmt::formatter<NAV::PressureModel> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] data Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::PressureModel& data, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(NAV::to_string(data), ctx);
    }
};