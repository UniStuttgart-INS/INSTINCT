#pragma once

#include <string>
#include <vector>

#include "uart/xplat/int.hpp"

/// \brief Defines for the specific version of the library.
#define API_MAJOR 1
#define API_MINOR 1
#define API_PATCH 1
#define API_REVISION 1

namespace uart
{
/// \brief Class for version information about the library.
class ApiVersion
{
  public:
    /// \brief Returns the major version of the library.
    ///
    /// \return The major version.
    static int major();

    /// \brief Returns the minor version of the library.
    ///
    /// \return The minor version
    static int minor();

    /// \brief Returns the patch version of the library.
    ///
    /// \return The patch version.
    static int patch();

    /// \brief Returns the revision version of the library.
    ///
    /// \return The revision version.
    static int revision();

    /// \brief Returns the full version of the library.
    ///
    /// \return The library's version in a string format.
    static std::string getVersion();
};

enum Endianness
{
    ENDIAN_LITTLE,
    ENDIAN_BIG,
    ENDIAN_UNKNOWN
};

/// \brief Converts two characters encoded in hex to a uint8_t.
///
/// \param[in] str Two characters string with hexadecimal encoding.
/// \return The converted value.
uint8_t toUint8FromHexStr(const char* str);

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
