#include "kvh/util/utilities.hpp"

#include <sstream>

#if defined _WINDOWS && PYTHON
    // TODO : This needs to function in Linux as well
    #include "dllvalidator.h"
#else
    #if defined __linux__
    #endif
#endif

#if _M_IX86 || __i386__ || __x86_64 || _WIN64
    // Compiling for x86 processor.
    #define KVH_LITTLE_ENDIAN 1
#elif __linux__
    // Don't know what processor we are compiling for but we have endian.h.
    #define KVH_HAVE_ENDIAN_H 1
    #include <endian.h>
#elif __BYTE_ORDER__
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        #define KVH_LITTLE_ENDIAN 1
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        #define KVH_BIG_ENDIAN 1
    #else
        #error "Unknown System"
    #endif
#else
    #error "Unknown System"
#endif

using namespace std;

namespace kvh
{
uint8_t toUint8FromHexStr(char const* str)
{
    uint8_t result;

    result = to_uint8_from_hexchar(str[0]) << 4;
    result += to_uint8_from_hexchar(str[1]);

    return result;
}

uint16_t stoh(uint16_t sensorOrdered)
{
#if KVH_LITTLE_ENDIAN
    uint16_t host;
    host = ((sensorOrdered >> 0) & 0xFF) * 0x0100;
    host += ((sensorOrdered >> 8) & 0xFF) * 0x0001;
    return host;
#elif KVH_BIG_ENDIAN
    return sensorOrdered;
#elif KVH_HAVE_ENDIAN_H
    return be16toh(sensorOrdered);
#else
    #error("Unknown system")
#endif
}

uint32_t stoh(uint32_t sensorOrdered)
{
#if KVH_LITTLE_ENDIAN
    uint32_t host;
    host = ((sensorOrdered >> 0) & 0xFF) * 0x01000000;
    host += ((sensorOrdered >> 8) & 0xFF) * 0x00010000;
    host += ((sensorOrdered >> 16) & 0xFF) * 0x00000100;
    host += ((sensorOrdered >> 24) & 0xFF) * 0x00000001;
    return host;
#elif KVH_BIG_ENDIAN
    return sensorOrdered;
#elif KVH_HAVE_ENDIAN_H
    return be32toh(sensorOrdered);
#else
    #error("Unknown system")
#endif
}

uint64_t stoh(uint64_t sensorOrdered)
{
#if KVH_LITTLE_ENDIAN
    uint64_t host;
    host = ((sensorOrdered >> 0) & 0xFF) * 0x0100000000000000;
    host += ((sensorOrdered >> 8) & 0xFF) * 0x0001000000000000;
    host += ((sensorOrdered >> 16) & 0xFF) * 0x0000010000000000;
    host += ((sensorOrdered >> 24) & 0xFF) * 0x0000000100000000;
    host += ((sensorOrdered >> 32) & 0xFF) * 0x0000000001000000;
    host += ((sensorOrdered >> 40) & 0xFF) * 0x0000000000010000;
    host += ((sensorOrdered >> 48) & 0xFF) * 0x0000000000000100;
    host += ((sensorOrdered >> 56) & 0xFF) * 0x0000000000000001;
    return host;
#elif KVH_BIG_ENDIAN
    return sensorOrdered;
#elif KVH_HAVE_ENDIAN_H
    return be64toh(sensorOrdered);
#else
    #error("Unknown system")
#endif
}

float stoh(float sensorOrdered)
{
    union
    {
        float rX;
        uint32_t ui32X;
    } uFloat;

    uFloat.rX = sensorOrdered;

#if KVH_LITTLE_ENDIAN
    uint32_t host;
    host = ((uFloat.ui32X >> 0) & 0xFF) * 0x01000000;
    host += ((uFloat.ui32X >> 8) & 0xFF) * 0x00010000;
    host += ((uFloat.ui32X >> 16) & 0xFF) * 0x00000100;
    host += ((uFloat.ui32X >> 24) & 0xFF) * 0x00000001;
    uFloat.ui32X = host;
    return (uFloat.rX);
#elif KVH_BIG_ENDIAN
    return sensorOrdered;
#elif KVH_HAVE_ENDIAN_H
    uFloat.ui32X = be32toh(uFloat.ui32X);
    return (uFloat.rX);
#else
    #error("Unknown system")
#endif
}

double stoh(double sensorOrdered)
{
    union
    {
        double rX;
        uint64_t ui64X;
    } uDouble;

    uDouble.rX = sensorOrdered;

#if KVH_LITTLE_ENDIAN
    uint64_t host;
    host = ((uDouble.ui64X >> 0) & 0xFF) * 0x0100000000000000;
    host += ((uDouble.ui64X >> 8) & 0xFF) * 0x0001000000000000;
    host += ((uDouble.ui64X >> 16) & 0xFF) * 0x0000010000000000;
    host += ((uDouble.ui64X >> 24) & 0xFF) * 0x0000000100000000;
    host += ((uDouble.ui64X >> 32) & 0xFF) * 0x0000000001000000;
    host += ((uDouble.ui64X >> 40) & 0xFF) * 0x0000000000010000;
    host += ((uDouble.ui64X >> 48) & 0xFF) * 0x0000000000000100;
    host += ((uDouble.ui64X >> 56) & 0xFF) * 0x0000000000000001;
    uDouble.ui64X = host;
    return (uDouble.rX);
#elif KVH_BIG_ENDIAN
    return sensorOrdered;
#elif KVH_HAVE_ENDIAN_H
    uDouble.ui64X = be64toh(uDouble.ui64X);
    return (uDouble.rX);
#else
    #error("Unknown system")
#endif
}

uint8_t countSetBits(uint8_t d)
{
    uint8_t count = 0;

    while (d)
    {
        d &= (d - 1);
        count++;
    }

    return count;
}

uint8_t to_uint8_from_hexchar(char c)
{
    if (c < ':')
        return c - '0';

    if (c < 'G')
        return c - '7';

    return c - 'W';
}

uint8_t to_uint8_from_hexstr(char const* str)
{
    return toUint8FromHexStr(str);
}

uint16_t to_uint16_from_hexstr(char const* str)
{
    uint16_t result;

    result = to_uint8_from_hexstr(str) << 8;
    result += to_uint8_from_hexstr(str + 2);

    return result;
}

} // namespace kvh
