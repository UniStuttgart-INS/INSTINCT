// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Units.hpp
/// @brief Units used by the system model parameters
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-20

#pragma once

#include <cstddef>
#include <string>

#include "util/Json.hpp"

namespace NAV
{

namespace Units
{

/// Possible Units for the Standard deviation of the acceleration due to user motion
enum class CovarianceAccelUnits : uint8_t
{
    m2_s3,    ///< [ m^2 / s^3 ]
    m_sqrts3, ///< [ m / √(s^3) ]
    COUNT,    ///< Amount of items in the enum
};

/// Possible Units for the Standard deviation of the clock phase drift
enum class CovarianceClkPhaseDriftUnits : uint8_t
{
    m2_s,    ///< [ m^2 / s ]
    m_sqrts, ///< [ m / √(s) ]
    COUNT,   ///< Amount of items in the enum
};

/// Possible Units for the Standard deviation of the clock frequency drift
enum class CovarianceClkFrequencyDriftUnits : uint8_t
{
    m2_s3,    ///< [ m^2 / s^3 ]
    m_sqrts3, ///< [ m / √(s^3) ]
    COUNT,    ///< Amount of items in the enum
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const CovarianceAccelUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, CovarianceAccelUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const CovarianceClkPhaseDriftUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, CovarianceClkPhaseDriftUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const CovarianceClkFrequencyDriftUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, CovarianceClkFrequencyDriftUnits& data);

} // namespace Units

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::CovarianceAccelUnits unit);

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::CovarianceClkPhaseDriftUnits unit);

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::CovarianceClkFrequencyDriftUnits unit);

/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::CovarianceAccelUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::CovarianceClkPhaseDriftUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::CovarianceClkFrequencyDriftUnits unit);

} // namespace NAV