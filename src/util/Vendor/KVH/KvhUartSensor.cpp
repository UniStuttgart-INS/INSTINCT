// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KvhUartSensor.hpp"

#include "KvhUtilities.hpp"
#include "util/Logger.hpp"

NAV::vendor::kvh::KvhUartSensor::KvhUartSensor(std::string name)
    : _name(std::move(name)), _buffer(uart::sensors::UartSensor::DefaultReadBufferSize)
{
    resetTracking();
}

void NAV::vendor::kvh::KvhUartSensor::resetTracking()
{
    _currentlyBuildingBinaryPacket = false;
    _currentlyBuildingAsciiPacket = false;

    _asciiEndChar1Found = false;
    _packetType = HeaderType::FMT_UNKNOWN;

    _buffer.resize(0);
}

NAV::vendor::kvh::KvhUartSensor::HeaderType NAV::vendor::kvh::KvhUartSensor::bFindImuHeader(uint8_t ui8Data)
{
    if (_eState == SM_IDLE)
    {
        if (ui8Data == ((HEADER_FMT_A >> 24U) & 0xFFU))
        // || ui8Data == ((HEADER_FMT_B >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_C >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_XBIT >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_XBIT2 >> 24U) & 0xFFU))
        {
            _eState = SM_H1;
        }
    }
    else if (_eState == SM_H1)
    {
        if (ui8Data == ((HEADER_FMT_A >> 16U) & 0xFFU))
        {
            _eState = SM_H2;
        }
        else
        {
            _eState = SM_IDLE;
        }
    }
    else if (_eState == SM_H2)
    {
        if (ui8Data == ((HEADER_FMT_A >> 8U) & 0xFFU))
        {
            _eState = SM_H3;
        }
        else if (ui8Data == ((HEADER_FMT_XBIT >> 8U) & 0xFFU))
        {
            _eState = SM_X3;
        }
        else
        {
            _eState = SM_IDLE;
        }
    }
    else if (_eState == SM_H3)
    {
        if (ui8Data == (HEADER_FMT_A & 0xFFU))
        {
            _eState = SM_IDLE;
            return HeaderType::FMT_A;
        }
        if (ui8Data == (HEADER_FMT_B & 0xFFU))
        {
            _eState = SM_IDLE;
            return HeaderType::FMT_B;
        }
        if (ui8Data == (HEADER_FMT_C & 0xFFU))
        {
            _eState = SM_IDLE;
            return HeaderType::FMT_C;
        }

        _eState = SM_IDLE;
    }
    else if (_eState == SM_X3)
    {
        if (ui8Data == (HEADER_FMT_XBIT & 0xFFU))
        {
            _eState = SM_IDLE;
            return HeaderType::FMT_XBIT;
        }
        if (ui8Data == (HEADER_FMT_XBIT2 & 0xFFU))
        {
            _eState = SM_IDLE;
            return HeaderType::FMT_XBIT2;
        }

        _eState = SM_IDLE;
    }
    else
    {
        _eState = SM_IDLE;
    }

    return HeaderType::FMT_UNKNOWN;
}

std::unique_ptr<uart::protocol::Packet> NAV::vendor::kvh::KvhUartSensor::findPacket(uint8_t dataByte)
{
    if (_buffer.size() == _buffer.capacity())
    {
        // Buffer is full
        resetTracking();
        LOG_ERROR("{}: Discarding current packet, because buffer is full.", _name);
    }

    auto binaryPacketType = bFindImuHeader(dataByte);
    if (binaryPacketType != HeaderType::FMT_UNKNOWN)
    {
        resetTracking();
        _packetType = binaryPacketType;
        _currentlyBuildingBinaryPacket = true;
        uint32_t header{};
        switch (binaryPacketType)
        {
        case HeaderType::FMT_A:
            header = uart::stoh(HEADER_FMT_A, ENDIANNESS);
            break;
        case HeaderType::FMT_B:
            header = uart::stoh(HEADER_FMT_B, ENDIANNESS);
            break;
        case HeaderType::FMT_C:
            header = uart::stoh(HEADER_FMT_C, ENDIANNESS);
            break;
        case HeaderType::FMT_XBIT:
            header = uart::stoh(HEADER_FMT_XBIT, ENDIANNESS);
            break;
        case HeaderType::FMT_XBIT2:
            header = uart::stoh(HEADER_FMT_XBIT2, ENDIANNESS);
            break;
        default:
            break;
        }
        _buffer.resize(4);
        memcpy(_buffer.data(), &header, 4);

        return nullptr;
    }

    if (!_currentlyBuildingAsciiPacket && !_currentlyBuildingBinaryPacket)
    {
        resetTracking();
        _currentlyBuildingAsciiPacket = true;
        _buffer.push_back(dataByte);
    }
    else if (_currentlyBuildingBinaryPacket)
    {
        _buffer.push_back(dataByte);

        if ((_packetType == HeaderType::FMT_A && _buffer.size() == 36)
            || (_packetType == HeaderType::FMT_B && _buffer.size() == 40)
            || (_packetType == HeaderType::FMT_C && _buffer.size() == 38)
            || (_packetType == HeaderType::FMT_XBIT && _buffer.size() == 11)
            || (_packetType == HeaderType::FMT_XBIT2 && _buffer.size() == 13))
        {
            // We have a possible binary packet!
            auto p = std::make_unique<uart::protocol::Packet>(_buffer, &_sensor);

            if (p->isValid())
            {
                // We have a valid binary packet!!!.
                LOG_DATA("{}: Valid binary packet: Type={}, Length={}", _name, fmt::underlying(_packetType), _buffer.size());
                resetTracking();
                return p;
            }
            // Invalid packet!
            LOG_ERROR("{}: Invalid binary packet: Type={}, Length={}", _name, fmt::underlying(_packetType), _buffer.size());
            resetTracking();
        }
        if (_buffer.size() >= 40)
        {
            resetTracking();
        }
    }
    else if (_currentlyBuildingAsciiPacket)
    {
        _buffer.push_back(dataByte);

        if (dataByte == ASCII_ESCAPE_CHAR)
        {
            resetTracking();
        }
        else if (dataByte == ASCII_END_CHAR_1)
        {
            _asciiEndChar1Found = true;
        }
        else if (_asciiEndChar1Found)
        {
            if (dataByte == ASCII_END_CHAR_2)
            {
                // We have a possible data packet
                auto p = std::make_unique<uart::protocol::Packet>(_buffer, &_sensor);

                if (p->isValid())
                {
                    // We have a valid ascii packet!!!.
                    LOG_DATA("{}: Valid ascii packet: {}", _name, p->datastr().substr(0, p->getRawDataLength() - 2));
                    resetTracking();
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("{}: Invalid ascii packet: {}", _name, p->datastr());
            }

            resetTracking();
        }

        if (_buffer.size() >= MAX_SIZE_ASCII_PACKET)
        {
            LOG_ERROR("{}: Buffer exceeded the Maximum Ascii Packet Size", _name);
            resetTracking();
        }
    }

    return nullptr;
}

void NAV::vendor::kvh::KvhUartSensor::packetFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData)
{
    auto* sensor = static_cast<KvhUartSensor*>(userData);

    for (size_t i = 0; i < data.size(); i++, sensor->_runningDataIndex++)
    {
        auto packetPointer = sensor->findPacket(data.at(i));

        if (packetPointer != nullptr)
        {
            uart::protocol::Packet packet = *packetPointer;
            dispatchPacket(dispatchPacketUserData, packet, sensor->_runningDataIndex, timestamp);
        }
    }
}

uart::protocol::Packet::Type NAV::vendor::kvh::KvhUartSensor::packetTypeFunction(const uart::protocol::Packet& packet)
{
    if (packet.getRawDataLength() < 1)
    {
        LOG_CRITICAL("Packet does not contain any data.");
    }

    // Check for carriage return and line feed
    if (packet.getRawData().at(packet.getRawDataLength() - 2) == uart::CARRIAGE_RETURN
        && packet.getRawData().at(packet.getRawDataLength() - 1) == uart::LINE_FEED)
    {
        return uart::protocol::Packet::Type::TYPE_ASCII;
    }

    uint32_t data_zero{};
    memcpy(&data_zero, packet.getRawData().data(), sizeof(uint32_t));
    data_zero = uart::stoh(data_zero, ENDIANNESS);

    if (data_zero == HEADER_FMT_A || data_zero == HEADER_FMT_B || data_zero == HEADER_FMT_C)
    {
        return uart::protocol::Packet::Type::TYPE_BINARY;
    }

    return uart::protocol::Packet::Type::TYPE_UNKNOWN;
}

bool NAV::vendor::kvh::KvhUartSensor::checksumFunction(const uart::protocol::Packet& packet)
{
    // minumum binary packet is 9 bytes
    if (packet.getRawDataLength() < 1)
    {
        return false;
    }

    if (packet.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        // Ascii does not have a checksum
        return true;
    }

    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        uint32_t checksumCalc = kvh::ui32CalcImuCRC(packet.getRawData());

        uint32_t checksumPacket = 0;
        memcpy(&checksumPacket, packet.getRawData().data() + packet.getRawDataLength() - sizeof(uint32_t), sizeof(uint32_t));
        checksumPacket = uart::stoh(checksumPacket, ENDIANNESS);

        return checksumPacket == checksumCalc;
    }

    LOG_CRITICAL("Can't calculate checksum of packet with unknown type");
    return false;
}

bool NAV::vendor::kvh::KvhUartSensor::isErrorFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}

bool NAV::vendor::kvh::KvhUartSensor::isResponseFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}