// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef HAS_UARTSENSOR_LIBRARY

    #include "uart/util/utilities.hpp"

    #include <sstream>
    #include <array>
    #include <algorithm>

    #if _M_IX86 || __i386__ || __x86_64 || _WIN64
        // Compiling for x86 processor.
        #define HOST_LITTLE_ENDIAN 1
    #elif __linux__
        // Don't know what processor we are compiling for but we have endian.h.
        #define HAVE_ENDIAN_H 1
        #include <endian.h>
    #elif __BYTE_ORDER__
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            #define HOST_LITTLE_ENDIAN 1
        #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            #define HOST_BIG_ENDIAN 1
        #else
            #error "Unknown System"
        #endif
    #else
        #error "Unknown System"
    #endif

namespace uart
{

template<typename T>
constexpr void SwapEndian(T& val)
{
    union U
    {
        T val;
        std::array<std::uint8_t, sizeof(T)> raw;
    } src{}, dst{};

    src.val = val;
    std::reverse_copy(src.raw.begin(), src.raw.end(), dst.raw.begin());
    val = dst.val;
}

uint16_t stoh(uint16_t sensorOrdered, Endianness sensorEndianness)
{
    #if HOST_LITTLE_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        SwapEndian<uint16_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HOST_BIG_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_LITTLE)
    {
        SwapEndian<uint16_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HAVE_ENDIAN_H
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        return be16toh(sensorOrdered);
    }
    return le16toh(sensorOrdered);
    #else
        #error("Unknown system")
    #endif
}

uint32_t stoh(uint32_t sensorOrdered, Endianness sensorEndianness)
{
    #if HOST_LITTLE_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        SwapEndian<uint32_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HOST_BIG_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_LITTLE)
    {
        SwapEndian<uint32_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HAVE_ENDIAN_H
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        return be32toh(sensorOrdered);
    }
    return le32toh(sensorOrdered);
    #else
        #error("Unknown system")
    #endif
}

uint64_t stoh(uint64_t sensorOrdered, Endianness sensorEndianness)
{
    #if HOST_LITTLE_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        SwapEndian<uint64_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HOST_BIG_ENDIAN
    if (sensorEndianness == Endianness::ENDIAN_LITTLE)
    {
        SwapEndian<uint64_t>(sensorOrdered);
    }
    return sensorOrdered;
    #elif HAVE_ENDIAN_H
    if (sensorEndianness == Endianness::ENDIAN_BIG)
    {
        return be64toh(sensorOrdered);
    }
    return le64toh(sensorOrdered);
    #else
        #error("Unknown system")
    #endif
}

float stoh(float sensorOrdered, Endianness sensorEndianness)
{
    union
    {
        float rX;
        uint32_t ui32X;
    } uFloat{};

    uFloat.rX = sensorOrdered;
    uFloat.ui32X = stoh(uFloat.ui32X, sensorEndianness);

    return (uFloat.rX);
}

double stoh(double sensorOrdered, Endianness sensorEndianness)
{
    union
    {
        double rX;
        uint64_t ui64X;
    } uDouble{};

    uDouble.rX = sensorOrdered;
    uDouble.ui64X = stoh(uDouble.ui64X, sensorEndianness);

    return (uDouble.rX);
}

uint8_t countSetBits(uint8_t d)
{
    uint8_t count = 0;

    while (d)
    {
        d &= static_cast<uint8_t>(d - 1U);
        count++;
    }

    return count;
}

uint8_t toUint8FromHexStr(char const* str)
{
    auto result = static_cast<uint8_t>(to_uint8_from_hexchar(str[0]) << 4U);
    result = static_cast<uint8_t>(result + to_uint8_from_hexchar(str[1]));

    return result;
}

uint8_t to_uint8_from_hexchar(char c)
{
    if (c < ':')
    {
        return static_cast<uint8_t>(c - '0');
    }

    if (c < 'G')
    {
        return static_cast<uint8_t>(c - '7');
    }

    return static_cast<uint8_t>(c - 'W');
}

uint8_t to_uint8_from_hexstr(char const* str)
{
    return toUint8FromHexStr(str);
}

uint16_t to_uint16_from_hexstr(char const* str)
{
    auto result = static_cast<uint16_t>(to_uint8_from_hexstr(str) << 8U);
    result = static_cast<uint16_t>(result + to_uint8_from_hexstr(str + 2));

    return result;
}

} // namespace uart

#endif