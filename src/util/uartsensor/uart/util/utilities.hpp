// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file packet.hpp
/// @brief Extract from the utilities implementation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_UARTSENSOR_LIBRARY

    #include <string>
    #include <vector>
    #include <cstdint>

namespace uart
{
/// Carriage Return \r
static constexpr unsigned int CARRIAGE_RETURN = 0x0D;
/// Line Feed \n
static constexpr unsigned int LINE_FEED = 0x0A;

/// @brief Stuct which contains the different types of Endianness
enum Endianness
{
    ENDIAN_LITTLE, ///< Little Endian
    ENDIAN_BIG,    ///< Big Endian
    ENDIAN_UNKNOWN ///< Endianness unknown
};

/// \brief Converts a 16-bit integer in sensor order to host order.
///
/// \param[in] sensorOrdered The 16-bit integer in sensor order.
/// \param[in] sensorEndianness The endianess of the sensor order
/// \return The value converted to host ordered.
uint16_t stoh(uint16_t sensorOrdered, Endianness sensorEndianness);

/// \brief Converts a 32-bit integer in sensor order to host order.
///
/// \param[in] sensorOrdered The 32-bit integer in sensor order.
/// \param[in] sensorEndianness The endianess of the sensor order
/// \return The value converted to host ordered.
uint32_t stoh(uint32_t sensorOrdered, Endianness sensorEndianness);

/// \brief Converts a 64-bit integer in sensor order to host order.
///
/// \param[in] sensorOrdered The 64-bit integer in sensor order.
/// \param[in] sensorEndianness The endianess of the sensor order
/// \return The value converted to host ordered.
uint64_t stoh(uint64_t sensorOrdered, Endianness sensorEndianness);

/// \brief Converts a float in sensor order to host order.
///
/// \param[in] sensorOrdered The float in sensor order.
/// \param[in] sensorEndianness The endianess of the sensor order
/// \return The value converted to host ordered.
float stoh(float sensorOrdered, Endianness sensorEndianness);

/// \brief Converts a double in sensor order to host order.
/// \param[in] sensorEndianness The endianess of the sensor order
///
/// \param[in] sensorOrdered The double in sensor order.
/// \return The value converted to host ordered.
double stoh(double sensorOrdered, Endianness sensorEndianness);

/// \brief Counts the number of bits set in the provided value.
///
/// \param[in] d The value to count the bits of.
/// \return The number of bits set.
uint8_t countSetBits(uint8_t d);

/// \brief Converts two characters encoded in hex to a uint8_t.
///
/// \param[in] str Two characters string with hexadecimal encoding.
/// \return The converted value.
uint8_t toUint8FromHexStr(const char* str);

/// \brief Converts the character encoded as a hexadecimal to a uint8_t.
///
/// \param[in] c The hexadecimal character to convert.
/// \return The converted value.
uint8_t to_uint8_from_hexchar(char c);

/// \brief Converts two characters encoded in hex to a uint8_t.
///
/// \param[in] str Two characters string with hexadecimal encoding.
/// \return The converted value.
uint8_t to_uint8_from_hexstr(const char* str);

/// \brief Converts four characters encoded in hex to a uint16_t.
///
/// \param[in] str Four characters string with hexadecimal encoding.
/// \return The converted value.
uint16_t to_uint16_from_hexstr(const char* str);

} // namespace uart

#endif