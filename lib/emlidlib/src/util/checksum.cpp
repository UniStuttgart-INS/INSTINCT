#include "er/util/checksum.hpp"

namespace er::util
{
std::pair<unsigned char, unsigned char> emlidChecksum::checksumUBX(unsigned char const data[], size_t length)
{
    unsigned char cka = 0, ckb = 0;

    for (size_t i = 2; i < length - 2; i++)
    {
        cka += (unsigned char)data[i];
        ckb += cka;
    }
    return std::make_pair(cka, ckb);
}

} // namespace er::util
