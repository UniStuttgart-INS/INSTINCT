#include "er/protocol/packet.hpp"
#include "er/util/utilities.hpp"
#include "er/util/checksum.hpp"

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
using namespace er::util;

namespace er::protocol::uart
{
char* ubstrtok(char* str, size_t& startIndex);

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
    if (_length < 1)
        throw std::runtime_error("Packet does not contain any data.");

    const unsigned char binary_indicator = 0XB5;
    // Since this is not a pointer use the functional cast for C++ rather than the C style cast
    const unsigned char data_zero = static_cast<unsigned char>(_data[0]); //unsigned char(_data[0]);

    if (_data[0] == '$')
        return TYPE_ASCII;
    if (data_zero == binary_indicator)
        return TYPE_BINARY;

    return TYPE_UNKNOWN;
}

bool Packet::isValid()
{
    if (_length < 9) // minumum binary packet is 9 bytes
        return false;

    if (type() == TYPE_BINARY)
    {
        auto checksum = emlidChecksum::checksumUBX(_data, _length);

        return _data[_length - 2] == checksum.first && _data[_length - 1] == checksum.second;
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

bool Packet::isAsciiAsync()
{
    // Pointer to the unique asynchronous data type identifier.
    char* pAT = (char*)_data + 1;

    if (strncmp(pAT, "GP", 2) == 0)
        return true;
    else if (strncmp(pAT, "GL", 2) == 0)
        return true;
    else if (strncmp(pAT, "GA", 2) == 0)
        return true;
    else if (strncmp(pAT, "GB", 2) == 0)
        return true;
    else if (strncmp(pAT, "GN", 2) == 0)
        return true;
    else
        return false;
}

char* ubstrtok(char* str, size_t& startIndex)
{
    size_t origIndex = startIndex;

    while (str[startIndex] != ',' && str[startIndex] != '*')
        startIndex++;

    str[startIndex++] = '\0';

    return str + origIndex;
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
        // Determine the location to start extracting.
        _curExtractLoc = 2;

    if (_curExtractLoc + numOfBytes > _length - 2)
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

    return f;
}
double Packet::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d;

    memcpy(&d, _data + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return d;
}

} // namespace er::protocol::uart
