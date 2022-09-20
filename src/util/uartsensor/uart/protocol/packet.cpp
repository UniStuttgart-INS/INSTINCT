// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef HAS_UARTSENSOR_LIBRARY

    #include "uart/protocol/packet.hpp"
    #include "uart/sensors/sensors.hpp"

    #include <cstdio>
    #include <cstdlib>
    #include <string>
    #include <stdexcept>

namespace uart::protocol
{
Packet::Packet(sensors::UartSensor* backReference)
    : _backReference(backReference) {}

Packet::Packet(std::vector<uint8_t> data, sensors::UartSensor* backReference)
    : _data(std::move(data)), _backReference(backReference) {}

Packet::Packet(const std::string& packet, sensors::UartSensor* backReference)
    : _data(packet.begin(), packet.end()), _backReference(backReference) {}

const std::vector<uint8_t>& Packet::getRawData() const
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
    _backReference = from._backReference;

    return *this;
}

std::string Packet::datastr() const
{
    return std::string(reinterpret_cast<const char*>(_data.data()), _data.size());
}

Packet::Type Packet::type() const
{
    if (_data.empty())
    {
        throw std::runtime_error("Packet does not contain any data.");
    }

    return _backReference->_packetTypeFunction(*this);
}

bool Packet::isValid() const
{
    return _backReference->_isValidFunction(*this);
}

bool Packet::isError() const
{
    return _backReference->_isErrorFunction(*this);
}

bool Packet::isResponse() const
{
    return _backReference->_isResponseFunction(*this);
}

void Packet::ensureCanExtract(size_t numOfBytes)
{
    if (_curExtractLoc == 0)
    {
        // Determine the location to start extracting.
        _curExtractLoc = _backReference->_packetHeaderLength;
    }

    if (_curExtractLoc + numOfBytes > _data.size())
    {
        // About to overrun data.
        throw std::out_of_range("Not enough data to extract");
    }
}

uint8_t Packet::extractUint8()
{
    ensureCanExtract(sizeof(uint8_t));

    uint8_t d = *(_data.data() + _curExtractLoc);

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

    return stoh(d, _backReference->_endianness);
}

uint32_t Packet::extractUint32()
{
    ensureCanExtract(sizeof(uint32_t));

    uint32_t d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint32_t));

    _curExtractLoc += sizeof(uint32_t);

    return stoh(d, _backReference->_endianness);
}

int32_t Packet::extractInt32()
{
    ensureCanExtract(sizeof(int32_t));

    int32_t d = 0;

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(int32_t));

    _curExtractLoc += sizeof(int32_t);

    return static_cast<int32_t>(stoh(static_cast<uint32_t>(d), _backReference->_endianness));
}

uint64_t Packet::extractUint64()
{
    ensureCanExtract(sizeof(uint64_t));

    uint64_t d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(uint64_t));

    _curExtractLoc += sizeof(uint64_t);

    return stoh(d, _backReference->_endianness);
}

float Packet::extractFloat()
{
    ensureCanExtract(sizeof(float));

    float f{};

    memcpy(&f, _data.data() + _curExtractLoc, sizeof(float));

    _curExtractLoc += sizeof(float);

    return stoh(f, _backReference->_endianness);
}

double Packet::extractDouble()
{
    ensureCanExtract(sizeof(double));

    double d{};

    memcpy(&d, _data.data() + _curExtractLoc, sizeof(double));

    _curExtractLoc += sizeof(double);

    return stoh(d, _backReference->_endianness);
}

} // namespace uart::protocol

#endif