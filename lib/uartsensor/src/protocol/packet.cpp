#include "uart/protocol/packet.hpp"
#include "uart/util/utilities.hpp"
#include "uart/util/checksum.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <queue>
#include <list>
#include <stdexcept>

namespace uart::protocol
{
Packet::Packet(unsigned char const* packet, size_t length)
    : _isPacketDataMine(true),
      _length(length),
      _data(new unsigned char[length])
{
    std::memcpy(_data, packet, length);
}

Packet::Packet(const std::string& packet)
    : _isPacketDataMine(true),
      _length(packet.size()),
      _data(new unsigned char[packet.size()])
{
    std::memcpy(_data, packet.c_str(), packet.size());
}

Packet::Packet(Packet const& toCopy)
    : _isPacketDataMine(true),
      _length(toCopy._length),
      _data(new unsigned char[toCopy._length])
{
    std::memcpy(_data, toCopy._data, toCopy._length);
}

Packet::~Packet()
{
    if (_isPacketDataMine)
    {
        delete[] _data;
    }
}

unsigned char* Packet::getRawData()
{
    return _data;
}

size_t Packet::getRawDataLength() const
{
    return _length;
}

Packet& Packet::operator=(Packet const& from)
{
    if (_isPacketDataMine)
    {
        delete[] _data;
    }

    _isPacketDataMine = true;
    _data = new unsigned char[from._length];
    _length = from._length;
    _curExtractLoc = from._curExtractLoc;

    std::memcpy(_data, from._data, from._length);

    return *this;
}

std::string Packet::datastr()
{
    return std::string(reinterpret_cast<char*>(_data), _length);
}

Packet::Type Packet::type()
{
    if (_length < 1)
    {
        throw std::runtime_error("Packet does not contain any data.");
    }

    const unsigned char binary_indicator = 0XB5;
    // Since this is not a pointer use the functional cast for C++ rather than the C style cast
    const auto data_zero = _data[0];

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

bool Packet::isValid()
{
    if (type() == TYPE_ASCII)
    {
        return util::Checksum::checksumASCII(_data, _length);
    }
    if (type() == TYPE_BINARY)
    {
        return util::Checksum::checksumBinary(_data, _length);
    }

    throw std::logic_error("Not implemented");
}

bool Packet::isError() const
{
    return _curExtractLoc > 10000;
}

bool Packet::isResponse() const
{
    return _curExtractLoc > 10000;
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
    {
        // Determine the location to start extracting.
        _curExtractLoc = 2;
    }

    if (_curExtractLoc + numOfBytes > _length - 2)
    {
        // About to overrun data.
        throw std::out_of_range("Not enough data to extract");
    }
}

uint8_t Packet::extractUint8()
{
    ensureCanExtract(sizeof(uint8_t));

    uint8_t d = *reinterpret_cast<uint8_t*>(_data + _curExtractLoc);

    _curExtractLoc += sizeof(uint8_t);

    return d;
}

int8_t Packet::extractInt8()
{
    ensureCanExtract(sizeof(int8_t));

    int8_t d = *reinterpret_cast<int8_t*>(_data + _curExtractLoc);

    _curExtractLoc += sizeof(int8_t);

    return d;
}

uint16_t Packet::extractUint16()
{
    ensureCanExtract(sizeof(uint16_t));

    uint16_t d{};

    memcpy(&d, _data + _curExtractLoc, sizeof(uint16_t));

    _curExtractLoc += sizeof(uint16_t);

    return stoh(d);
}

uint32_t Packet::extractUint32()
{
    ensureCanExtract(sizeof(uint32_t));

    uint32_t d{};

    memcpy(&d, _data + _curExtractLoc, sizeof(uint32_t));

    _curExtractLoc += sizeof(uint32_t);

    return stoh(d);
}

uint64_t Packet::extractUint64()
{
    ensureCanExtract(sizeof(uint64_t));

    uint64_t d{};

    memcpy(&d, _data + _curExtractLoc, sizeof(uint64_t));

    _curExtractLoc += sizeof(uint64_t);

    return stoh(d);
}

float Packet::extractFloat()
{
    ensureCanExtract(sizeof(float));

    float f{};

    memcpy(&f, _data + _curExtractLoc, sizeof(float));

    _curExtractLoc += sizeof(float);

    return stoh(f);
}
double Packet::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d{};

    memcpy(&d, _data + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return stoh(d);
}

} // namespace uart::protocol
