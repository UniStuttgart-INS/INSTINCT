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
#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{

/// Temperature Model parameters
class TemperatureModel
{
  public:
    /// Available temperature Models
    enum Model : uint8_t
    {
        None,  ///< No temperature model
        Const, ///< Constant value
        ISA,   ///< ICAO Standard Atmosphere
        GPT2,  ///< GPT2
        GPT3,  ///< GPT3
        COUNT, ///< Amount of items in the enum
    };

    /// @brief Constructor
    /// @param model Model to use
    TemperatureModel(Model model) : _model(model) {} // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)

    /// @brief Calculates the absolute temperature
    /// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
    /// @return The absolute temperature in [K]
    [[nodiscard]] double calcAbsoluteTemperature(double altitudeMSL) const;

    /// @brief Shows a ComboBox to select the temperature model
    /// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
    /// @param[in] temperatureModel Reference to the temperature model to select
    friend bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel);

  private:
    /// Selected model
    Model _model = Model::ISA;

    /// Temperature for the constant temperature model [K]
    double _constantTemperature = 290.0;

    friend constexpr bool operator==(const TemperatureModel& lhs, const TemperatureModel& rhs);
    friend constexpr bool operator==(const TemperatureModel& lhs, const TemperatureModel::Model& rhs);
    friend constexpr bool operator==(const TemperatureModel::Model& lhs, const TemperatureModel& rhs);
    friend bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel);
    friend const char* to_string(const TemperatureModel& temperatureModel);
    friend void to_json(json& j, const TemperatureModel& obj);
    friend void from_json(const json& j, TemperatureModel& obj);
};

/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TemperatureModel& lhs, const TemperatureModel& rhs)
{
    if (lhs._model == rhs._model)
    {
        if (lhs._model == TemperatureModel::Const) { return lhs._constantTemperature == rhs._constantTemperature; }
        return true;
    }
    return false;
}

/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TemperatureModel& lhs, const TemperatureModel::Model& rhs) { return lhs._model == rhs; }
/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TemperatureModel::Model& lhs, const TemperatureModel& rhs) { return lhs == rhs._model; }

/// @brief Shows a ComboBox to select the temperature model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] temperatureModel Reference to the temperature model to select
bool ComboTemperatureModel(const char* label, TemperatureModel& temperatureModel);

/// @brief Converts the enum to a string
/// @param[in] temperatureModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(const TemperatureModel& temperatureModel);
/// @brief Converts the enum to a string
/// @param[in] temperatureModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(TemperatureModel::Model temperatureModel);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const TemperatureModel& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, TemperatureModel& obj);

} // namespace NAV

#ifndef DOXYGEN_IGNORE

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

#endif