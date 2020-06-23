#include "EmlidPacket.hpp"

#include <stdexcept>

NAV::Emlid::EmlidPacket::EmlidPacket(unsigned char const* packet, size_t length)
{
    setData(packet, length);
}

void NAV::Emlid::EmlidPacket::setData(unsigned char const* packet, size_t length)
{
    _data.clear();
    std::copy(&packet[0], &packet[length], std::back_inserter(_data));
}

const unsigned char* NAV::Emlid::EmlidPacket::getRawData() const
{
    return _data.data();
}

size_t NAV::Emlid::EmlidPacket::getRawDataLength() const
{
    return _data.size();
}

std::string NAV::Emlid::EmlidPacket::datastr()
{
    return std::string(reinterpret_cast<char*>(_data.data()), _data.size());
}

NAV::Emlid::EmlidPacket::Type NAV::Emlid::EmlidPacket::type() const
{
    if (_data.empty())
    {
        throw std::runtime_error("Packet does not contain any data.");
    }

    const unsigned char binary_indicator = 0X45;
    // Since this is not a pointer use the functional cast for C++ rather than the C style cast
    const auto data_zero = static_cast<unsigned char>(_data[0]); //unsigned char(_data[0]);

    if (_data[0] == '$')
    {
        return TYPE_ASCII;
    }
    if (data_zero == binary_indicator)
    {
        return TYPE_BINARY;
    }

    return TYPE_UNKNOWN;
}

void NAV::Emlid::EmlidPacket::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
    {
        // Determine the location to start extracting.
        _curExtractLoc = 2;
    }

    if (_curExtractLoc + numOfBytes > _data.size() - 2)
    {
        // About to overrun data.
        throw std::out_of_range("Not enough data to extract");
    }
}

uint8_t NAV::Emlid::EmlidPacket::extractUint8()
{
    ensureCanExtract(sizeof(uint8_t));

    uint8_t d = *reinterpret_cast<uint8_t*>(_data.data() + _curExtractLoc);

    _curExtractLoc += sizeof(uint8_t);

    return d;
}

int8_t NAV::Emlid::EmlidPacket::extractInt8()
{
    ensureCanExtract(sizeof(int8_t));

    int8_t d = *reinterpret_cast<int8_t*>(_data.data() + _curExtractLoc);

    _curExtractLoc += sizeof(int8_t);

    return d;
}

uint16_t NAV::Emlid::EmlidPacket::extractUint16()
{
    ensureCanExtract(sizeof(uint16_t));

    uint16_t d = 0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint16_t));

    _curExtractLoc += sizeof(uint16_t);

    return d;
}

uint32_t NAV::Emlid::EmlidPacket::extractUint32()
{
    ensureCanExtract(sizeof(uint32_t));

    uint32_t d = 0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint32_t));

    _curExtractLoc += sizeof(uint32_t);

    return d;
}

int32_t NAV::Emlid::EmlidPacket::extractInt32()
{
    ensureCanExtract(sizeof(int32_t));

    int32_t d = 0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(int32_t));

    _curExtractLoc += sizeof(int32_t);

    return d;
}

uint64_t NAV::Emlid::EmlidPacket::extractUint64()
{
    ensureCanExtract(sizeof(uint64_t));

    uint64_t d = 0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint64_t));

    _curExtractLoc += sizeof(uint64_t);

    return d;
}

float NAV::Emlid::EmlidPacket::extractFloat()
{
    ensureCanExtract(sizeof(float));

    float f = 0.0F;

    memcpy(&f, _data.data() + _curExtractLoc, sizeof(float));

    _curExtractLoc += sizeof(float);

    return f;
}
double NAV::Emlid::EmlidPacket::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d = 0.0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return d;
}