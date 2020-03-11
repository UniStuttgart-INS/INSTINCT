#include "ub/util/checksum.hpp"

namespace ub::util
{
std::pair<unsigned char, unsigned char> ubloxChecksum::checksumUBX(unsigned char const data[], size_t length)
{
    unsigned char cka = 0, ckb = 0;

    for (size_t i = 2; i < length - 2; i++)
    {
        cka += (unsigned char)data[i];
        ckb += cka;
    }
    return std::make_pair(cka, ckb);
}

uint8_t ubloxChecksum::checksumNMEA(char const data[], size_t length)
{
    int calcChecksum = 0;
    for (size_t i = 1; i < length - 5; i++)
    {
        calcChecksum ^= data[i];
    }
    return calcChecksum;
}

} // namespace ub::util
