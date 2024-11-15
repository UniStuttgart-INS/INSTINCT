// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Units.hpp
/// @brief Units used by INS
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-21

#pragma once

#include <cstddef>
#include <string>

#include "Navigation/Transformations/Units.hpp"
#include "util/Eigen.hpp"
#include "util/Json.hpp"

namespace NAV
{

namespace Units
{

/// Possible units to specify an accelerometer with
enum class ImuAccelerometerUnits : uint8_t
{
    m_s2,  ///< [m/s^2]
    COUNT, ///< Amount of items in the enum
};

/// Possible units to specify an gyroscope bias with
enum class ImuGyroscopeUnits : uint8_t
{
    rad_s, ///< [rad/s]
    deg_s, ///< [deg/s]
    COUNT, ///< Amount of items in the enum
};

/// Possible units to specify an accelerometer noise
enum class ImuAccelerometerNoiseUnits : uint8_t
{
    m_s2_sqrts, ///< [m/s^2/sqrt(s)] (Standard deviation)
    m_s2_sqrth, ///< [m/s^2/sqrt(h)] (Standard deviation)
    COUNT,      ///< Amount of items in the enum
};

/// Possible units to specify an gyro noise
enum class ImuGyroscopeNoiseUnits : uint8_t
{
    rad_s_sqrts, ///< [rad/s/sqrt(s)] (Standard deviation)
    rad_s_sqrth, ///< [rad/s/sqrt(h)] (Standard deviation)
    deg_s_sqrts, ///< [deg/s/sqrt(s)] (Standard deviation)
    deg_s_sqrth, ///< [deg/s/sqrt(h)] (Standard deviation)
    COUNT,       ///< Amount of items in the enum
};

/// Possible units to specify an accelerometer IRW
enum class ImuAccelerometerIRWUnits : uint8_t
{
    m_s3_sqrts, ///< [m/s^3/sqrt(s)] (Standard deviation)
    m_s3_sqrth, ///< [m/s^3/sqrt(h)] (Standard deviation)
    COUNT,      ///< Amount of items in the enum
};

/// Possible units to specify an gyro RW
enum class ImuGyroscopeIRWUnits : uint8_t
{
    rad_s2_sqrts, ///< [rad/s^2/sqrt(s)] (Standard deviation)
    rad_s2_sqrth, ///< [rad/s^2/sqrt(h)] (Standard deviation)
    deg_s2_sqrts, ///< [deg/s^2/sqrt(s)] (Standard deviation)
    deg_s2_sqrth, ///< [deg/s^2/sqrt(h)] (Standard deviation)
    COUNT,        ///< Amount of items in the enum
};

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuAccelerometerUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuAccelerometerUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuGyroscopeUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuGyroscopeUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuAccelerometerNoiseUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuAccelerometerNoiseUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuGyroscopeNoiseUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuGyroscopeNoiseUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuAccelerometerIRWUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuAccelerometerIRWUnits& data);

/// @brief Converts the provided data into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const ImuGyroscopeIRWUnits& data);
/// @brief Converts the provided json object into the data object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, ImuGyroscopeIRWUnits& data);

} // namespace Units

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuAccelerometerUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerUnits::m_s2:
        return value;
    case Units::ImuAccelerometerUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuGyroscopeUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeUnits::rad_s:
        return value;
    case Units::ImuGyroscopeUnits::deg_s:
        return deg2rad(value);
    case Units::ImuGyroscopeUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuAccelerometerNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerNoiseUnits::m_s2_sqrts:
        return value;
    case Units::ImuAccelerometerNoiseUnits::m_s2_sqrth:
        return value / 60.0;
    case Units::ImuAccelerometerNoiseUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuGyroscopeNoiseUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeNoiseUnits::rad_s_sqrts:
        return value;
    case Units::ImuGyroscopeNoiseUnits::rad_s_sqrth:
        return value / 60.0;
    case Units::ImuGyroscopeNoiseUnits::deg_s_sqrts:
        return deg2rad(value);
    case Units::ImuGyroscopeNoiseUnits::deg_s_sqrth:
        return deg2rad(value) / 60.0;
    case Units::ImuGyroscopeNoiseUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuAccelerometerIRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuAccelerometerIRWUnits::m_s3_sqrts:
        return value;
    case Units::ImuAccelerometerIRWUnits::m_s3_sqrth:
        return value / 60.0;
    case Units::ImuAccelerometerIRWUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the value depending on the unit provided
/// @param[in] value Value to convert
/// @param[in] unit Unit the value is in
/// @return Value in unit of the first item in the Unit enum
template<typename Derived>
[[nodiscard]] typename Derived::PlainObject convertUnit(const Eigen::MatrixBase<Derived>& value, Units::ImuGyroscopeIRWUnits unit)
{
    switch (unit)
    {
    case Units::ImuGyroscopeIRWUnits::rad_s2_sqrts:
        return value;
    case Units::ImuGyroscopeIRWUnits::rad_s2_sqrth:
        return value / 60.0;
    case Units::ImuGyroscopeIRWUnits::deg_s2_sqrts:
        return deg2rad(value);
    case Units::ImuGyroscopeIRWUnits::deg_s2_sqrth:
        return deg2rad(value) / 60.0;
    case Units::ImuGyroscopeIRWUnits::COUNT:
        break;
    }
    return value;
}

/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuAccelerometerUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuGyroscopeUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuAccelerometerNoiseUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuGyroscopeNoiseUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuAccelerometerIRWUnits unit);
/// @brief Converts the unit into a string
/// @param[in] unit Unit
[[nodiscard]] std::string to_string(Units::ImuGyroscopeIRWUnits unit);

} // namespace NAV