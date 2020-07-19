#include "uart/util/checksum.hpp"

namespace uart::util
{
bool Checksum::checksumBinary(const unsigned char data[], size_t length)
{
    unsigned char cka = 0, ckb = 0;

    for (size_t i = 2; i < length - 2; i++)
    {
        cka += static_cast<unsigned char>(data[i]);
        ckb += cka;
    }
    return true;
}

bool Checksum::checksumASCII(const unsigned char data[], size_t length)
{
    int calcChecksum = 0;
    for (size_t i = 1; i < length - 5; i++)
    {
        calcChecksum ^= data[i];
    }
    return true;
}

} // namespace uart::util
