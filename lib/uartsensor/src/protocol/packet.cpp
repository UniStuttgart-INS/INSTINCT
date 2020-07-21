#include "uart/protocol/packet.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <stdexcept>

namespace uart::protocol
{
Packet::Packet(std::vector<uint8_t> data)
    : _data(std::move(data)) {}

Packet::Packet(const std::string& packet)
    : _data(packet.begin(), packet.end()) {}

Packet::Packet(Packet const& toCopy)
    : _data(toCopy._data) {}

const std::vector<uint8_t>& Packet::getRawData()
{
    return _data;
}

size_t Packet::getRawDataLength() const
{
    return _data.size();
}

Packet& Packet::operator=(Packet const& from)
{
    if (this == &from)
    {
        return *this;
    }

    _data = from._data;
    _curExtractLoc = from._curExtractLoc;
    _endianness = from._endianness;

    _packetTypeFunction = from._packetTypeFunction;
    _checksumFunction = from._checksumFunction;
    _isErrorFunction = from._isErrorFunction;
    _isResponseFunction = from._isResponseFunction;
    _packageHeaderLength = from._packageHeaderLength;
    _packageEndLength = from._packageEndLength;

    return *this;
}

std::string Packet::datastr()
{
    return std::string(reinterpret_cast<char*>(_data.data()), _data.size());
}

Packet::Type Packet::type()
{
    if (_data.empty())
    {
        throw std::runtime_error("Packet does not contain any data.");
    }

    return _packetTypeFunction(*this);
}

bool Packet::isValid()
{
    return _checksumFunction(*this);
}

bool Packet::isError() const
{
    return _isErrorFunction(*this);
}

bool Packet::isResponse() const
{
    return _isResponseFunction(*this);
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
    {
        // Determine the location to start extracting.
        _curExtractLoc = _packageHeaderLength;
    }

    if (_curExtractLoc + numOfBytes > _data.size() - _packageEndLength)
    {
        // About to overrun data.
        throw std::out_of_range("Not enough data to extract");
    }
}

uint8_t Packet::extractUint8()
{
    ensureCanExtract(sizeof(uint8_t));

    uint8_t d = *reinterpret_cast<uint8_t*>(_data.data() + _curExtractLoc);

    _curExtractLoc += sizeof(uint8_t);

    return d;
}

int8_t Packet::extractInt8()
{
    ensureCanExtract(sizeof(int8_t));

    int8_t d = *reinterpret_cast<int8_t*>(_data.data() + _curExtractLoc);

    _curExtractLoc += sizeof(int8_t);

    return d;
}

uint16_t Packet::extractUint16()
{
    ensureCanExtract(sizeof(uint16_t));

    uint16_t d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint16_t));

    _curExtractLoc += sizeof(uint16_t);

    return stoh(d, _endianness);
}

uint32_t Packet::extractUint32()
{
    ensureCanExtract(sizeof(uint32_t));

    uint32_t d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint32_t));

    _curExtractLoc += sizeof(uint32_t);

    return stoh(d, _endianness);
}

uint64_t Packet::extractUint64()
{
    ensureCanExtract(sizeof(uint64_t));

    uint64_t d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint64_t));

    _curExtractLoc += sizeof(uint64_t);

    return stoh(d, _endianness);
}

float Packet::extractFloat()
{
    ensureCanExtract(sizeof(float));

    float f{};

    memcpy(&f, _data.data() + _curExtractLoc, sizeof(float));

    _curExtractLoc += sizeof(float);

    return stoh(f, _endianness);
}

double Packet::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return stoh(d, _endianness);
}

} // namespace uart::protocol
