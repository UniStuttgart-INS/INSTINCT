// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Units.hpp
/// @brief Units used by GNSS positioning
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2025-02-18

#pragma once

#include <cstddef>
#include <string>
#include <Eigen/Core>

#include "util/Json.hpp"

namespace NAV
{

namespace Units
{

/// Possible units for the uncertainty of the position
enum class PositionUncertaintyUnits : uint8_t
{
    meter,  ///< Standard deviation [m, m, m]
    meter2, ///< Variance [m^2, m^2, m^2]
    COUNT,  ///< Amount of items in the enum
};

/// Possible units for the uncertainty of the velocity
enum class VelocityUncertaintyUnits : uint8_t
{
    m_s,   ///< Standard deviation [m/s]
    m2_s2, ///< Variance [m^2/s^2]
    COUNT, ///< Amount of items in the enum
};

/// Possible units for the uncertainty of the attitude
enum class AttitudeUncertaintyUnits : uint8_t
{
    rad,   ///< Standard deviation [radian]
    deg,   ///< Standard deviation [degree]
    rad2,  ///< Variance [radian^2]
    deg2,  ///< Variance [degree^2]
    COUNT, ///< Amount of items in the enum
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const PositionUncertaintyUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, PositionUncertaintyUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const VelocityUncertaintyUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, VelocityUncertaintyUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const AttitudeUncertaintyUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, AttitudeUncertaintyUnits& data);

} // namespace Units

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::PositionUncertaintyUnits unit);
/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] Eigen::Vector3d convertUnit(const Eigen::Vector3d& value, Units::PositionUncertaintyUnits unit);

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::VelocityUncertaintyUnits unit);
/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] Eigen::Vector3d convertUnit(const Eigen::Vector3d& value, Units::VelocityUncertaintyUnits unit);

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] double convertUnit(const double& value, Units::AttitudeUncertaintyUnits unit);
/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
[[nodiscard]] Eigen::Vector3d convertUnit(const Eigen::Vector3d& value, Units::AttitudeUncertaintyUnits unit);

/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::PositionUncertaintyUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::VelocityUncertaintyUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::AttitudeUncertaintyUnits unit);

} // namespace NAV