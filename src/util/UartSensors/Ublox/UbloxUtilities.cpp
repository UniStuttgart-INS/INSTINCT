#include "UbloxUtilities.hpp"

std::pair<uint8_t, uint8_t> NAV::sensors::ublox::checksumUBX(const std::vector<uint8_t>& data)
{
    uint8_t cka = 0;
    uint8_t ckb = 0;

    for (size_t i = 2; i < data.size() - 2; i++)
    {
        cka += data.at(i);
        ckb += cka;
    }
    return std::make_pair(cka, ckb);
}

uint8_t NAV::sensors::ublox::checksumNMEA(const std::vector<uint8_t>& data)
{
    uint8_t calcChecksum = 0;
    for (size_t i = 1; i < data.size() - 5; i++)
    {
        calcChecksum ^= data.at(i);
    }
    return calcChecksum;
}