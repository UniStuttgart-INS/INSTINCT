#include "kvh/protocol/packet.hpp"
#include "kvh/util/utilities.hpp"
#include "kvh/util/checksum.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <queue>
#include <list>
#include <stdexcept>

#include <cstring>

#define NEXT                                 \
    result = getNextData(_data, parseIndex); \
    if (result == NULL)                      \
        return;

#define ATOFF static_cast<float>(std::atof(result))
#define ATOFD std::atof(result)
#define ATOU32 static_cast<uint32_t>(std::atoi(result))
#define ATOU16X ((uint16_t)std::strtol(result, NULL, 16))
#define ATOU16 static_cast<uint16_t>(std::atoi(result))
#define ATOU8 static_cast<uint8_t>(std::atoi(result))

using namespace std;
using namespace kvh::util;

namespace kvh::protocol::uart
{
constexpr uint32_t Packet::HEADER_FMT_A;
constexpr uint32_t Packet::HEADER_FMT_B;
constexpr uint32_t Packet::HEADER_FMT_C;
constexpr uint32_t Packet::HEADER_FMT_XBIT;
constexpr uint32_t Packet::HEADER_FMT_XBIT2;

Packet::Packet()
    : _isPacketDataMine(false),
      _length(0),
      _data(NULL)
{
}

Packet::Packet(unsigned char const* packet, size_t length)
    : _isPacketDataMine(true),
      _length(length),
      _data(new unsigned char[length]),
      _curExtractLoc(0)
{
    std::memcpy(_data, packet, length);
}

Packet::Packet(string packet)
    : _isPacketDataMine(true),
      _length(packet.size()),
      _data(new unsigned char[packet.size()]),
      _curExtractLoc(0)
{
    std::memcpy(_data, packet.c_str(), packet.size());
}

Packet::Packet(Packet const& toCopy)
    : _isPacketDataMine(true),
      _length(toCopy._length),
      _data(new unsigned char[toCopy._length]),
      _curExtractLoc(0)
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

size_t Packet::getRawDataLength()
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

string Packet::datastr()
{
    return string((char*)_data, _length);
}

Packet::Type Packet::type()
{
    if (_length < 4)
        throw std::runtime_error("Packet does not contain any data.");

    // Since this is not a pointer use the functional cast for C++ rather than the C style cast
    uint32_t data_zero;
    memcpy(&data_zero, _data, 4);
    data_zero = stoh(data_zero);

    if (data_zero == HEADER_FMT_A)
        return TYPE_BINARY_FMT_A;
    if (data_zero == HEADER_FMT_B)
        return TYPE_BINARY_FMT_B;
    if (data_zero == HEADER_FMT_C)
        return TYPE_BINARY_FMT_C;

    return TYPE_UNKNOWN;
}
bool Packet::isValid()
{
    if (_length < 9) // minumum binary packet is 9 bytes
        return false;

    if (type() == TYPE_ASCII)
    {
        // Ascii does not have a checksum, just check if it has the carriage return and line feed
        if (_data[_length - 2] == 0x0D && _data[_length - 1] == 0x0A)
        {
            return true;
        }
        // Don't know what we have.
        return false;
    }
    else if (type() == TYPE_BINARY_FMT_A || type() == TYPE_BINARY_FMT_B || type() == TYPE_BINARY_FMT_C)
    {
        uint32_t checksumCalc = kvhChecksum::ui32CalcImuCRC(_data, _length - 4);

        uint32_t checksumPacket;
        memcpy(&checksumPacket, _data + _length - 4, 4);
        checksumPacket = stoh(checksumPacket);

        return checksumPacket == checksumCalc;
    }
    else
    {
        throw std::logic_error("Not implemented");
    }
}

bool Packet::isError()
{
    return std::strncmp((char*)_data + 3, "ERR", 3) == 0;
}

bool Packet::isResponse()
{
    if (std::strncmp((char*)_data + 3, "WRG", 3) == 0)
        return true;

    return false;
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
        // Determine the location to start extracting.
        _curExtractLoc = 4;

    if (_curExtractLoc + numOfBytes > _length - 4)
        // About to overrun data.
        throw std::out_of_range("Not enough data to extract");
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

    uint16_t d;

    memcpy(&d, _data + _curExtractLoc, sizeof(uint16_t));

    _curExtractLoc += sizeof(uint16_t);

    return stoh(d);
}

uint32_t Packet::extractUint32()
{
    ensureCanExtract(sizeof(uint32_t));

    uint32_t d;

    memcpy(&d, _data + _curExtractLoc, sizeof(uint32_t));

    _curExtractLoc += sizeof(uint32_t);

    return stoh(d);
}

uint64_t Packet::extractUint64()
{
    ensureCanExtract(sizeof(uint64_t));

    uint64_t d;

    memcpy(&d, _data + _curExtractLoc, sizeof(uint64_t));

    _curExtractLoc += sizeof(uint64_t);

    return stoh(d);
}

float Packet::extractFloat()
{
    ensureCanExtract(sizeof(float));

    float f;

    memcpy(&f, _data + _curExtractLoc, sizeof(float));

    _curExtractLoc += sizeof(float);

    return stoh(f);
}
double Packet::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d;

    memcpy(&d, _data + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return stoh(d);
}

} // namespace kvh::protocol::uart
