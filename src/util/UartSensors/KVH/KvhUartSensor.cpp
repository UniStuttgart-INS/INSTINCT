#include "KvhUartSensor.hpp"

#include "KvhUtilities.hpp"
#include "util/Logger.hpp"

NAV::sensors::kvh::KvhUartSensor::KvhUartSensor()
    : _buffer(uart::sensors::UartSensor::DefaultReadBufferSize)
{
    resetTracking();
}

void NAV::sensors::kvh::KvhUartSensor::resetTracking()
{
    currentlyBuildingBinaryPacket = false;
    currentlyBuildingAsciiPacket = false;

    asciiEndChar1Found = false;
    packetType = HeaderType::FMT_UNKNOWN;

    _buffer.resize(0);
}

NAV::sensors::kvh::KvhUartSensor::HeaderType NAV::sensors::kvh::KvhUartSensor::bFindImuHeader(uint8_t ui8Data)
{
    if (eState == SM_IDLE)
    {
        if (ui8Data == ((HEADER_FMT_A >> 24U) & 0xFFU))
        // || ui8Data == ((HEADER_FMT_B >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_C >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_XBIT >> 24U) & 0xFFU)
        // || ui8Data == ((HEADER_FMT_XBIT2 >> 24U) & 0xFFU))
        {
            eState = SM_H1;
        }
    }
    else if (eState == SM_H1)
    {
        if (ui8Data == ((HEADER_FMT_A >> 16U) & 0xFFU))
        {
            eState = SM_H2;
        }
        else
        {
            eState = SM_IDLE;
        }
    }
    else if (eState == SM_H2)
    {
        if (ui8Data == ((HEADER_FMT_A >> 8U) & 0xFFU))
        {
            eState = SM_H3;
        }
        else if (ui8Data == ((HEADER_FMT_XBIT >> 8U) & 0xFFU))
        {
            eState = SM_X3;
        }
        else
        {
            eState = SM_IDLE;
        }
    }
    else if (eState == SM_H3)
    {
        if (ui8Data == (HEADER_FMT_A & 0xFFU))
        {
            eState = SM_IDLE;
            return HeaderType::FMT_A;
        }
        if (ui8Data == (HEADER_FMT_B & 0xFFU))
        {
            eState = SM_IDLE;
            return HeaderType::FMT_B;
        }
        if (ui8Data == (HEADER_FMT_C & 0xFFU))
        {
            eState = SM_IDLE;
            return HeaderType::FMT_C;
        }

        eState = SM_IDLE;
    }
    else if (eState == SM_X3)
    {
        if (ui8Data == (HEADER_FMT_XBIT & 0xFFU))
        {
            eState = SM_IDLE;
            return HeaderType::FMT_XBIT;
        }
        if (ui8Data == (HEADER_FMT_XBIT2 & 0xFFU))
        {
            eState = SM_IDLE;
            return HeaderType::FMT_XBIT2;
        }

        eState = SM_IDLE;
    }
    else
    {
        eState = SM_IDLE;
    }

    return HeaderType::FMT_UNKNOWN;
}

std::unique_ptr<uart::protocol::Packet> NAV::sensors::kvh::KvhUartSensor::findPacket(uint8_t dataByte)
{
    if (_buffer.size() == _buffer.capacity())
    {
        // Buffer is full
        resetTracking();
        LOG_ERROR("Discarding current packet, because buffer is full.");
    }

    auto binaryPacketType = bFindImuHeader(dataByte);
    if (binaryPacketType != HeaderType::FMT_UNKNOWN)
    {
        resetTracking();
        packetType = binaryPacketType;
        currentlyBuildingBinaryPacket = true;
        uint32_t header{};
        switch (binaryPacketType)
        {
        case HeaderType::FMT_A:
            header = uart::stoh(HEADER_FMT_A, endianness);
            break;
        case HeaderType::FMT_B:
            header = uart::stoh(HEADER_FMT_B, endianness);
            break;
        case HeaderType::FMT_C:
            header = uart::stoh(HEADER_FMT_C, endianness);
            break;
        case HeaderType::FMT_XBIT:
            header = uart::stoh(HEADER_FMT_XBIT, endianness);
            break;
        case HeaderType::FMT_XBIT2:
            header = uart::stoh(HEADER_FMT_XBIT2, endianness);
            break;
        default:
            break;
        }
        _buffer.resize(4);
        memcpy(_buffer.data(), &header, 4);

        return nullptr;
    }

    if (!currentlyBuildingAsciiPacket && !currentlyBuildingBinaryPacket)
    {
        resetTracking();
        currentlyBuildingAsciiPacket = true;
        _buffer.push_back(dataByte);
    }
    else if (currentlyBuildingBinaryPacket)
    {
        _buffer.push_back(dataByte);

        if ((packetType == HeaderType::FMT_A && _buffer.size() == 36)
            || (packetType == HeaderType::FMT_B && _buffer.size() == 40)
            || (packetType == HeaderType::FMT_C && _buffer.size() == 38)
            || (packetType == HeaderType::FMT_XBIT && _buffer.size() == 11)
            || (packetType == HeaderType::FMT_XBIT2 && _buffer.size() == 13))
        {
            // We have a possible binary packet!
            auto p = std::make_unique<uart::protocol::Packet>(_buffer, &sensor);

            if (p->isValid())
            {
                // We have a valid binary packet!!!.
                LOG_TRACE("Valid binary packet: Type={}, Length={}", packetType, _buffer.size());
                resetTracking();
                return p;
            }
            // Invalid packet!
            LOG_ERROR("Invalid binary packet: Type={}, Length={}", packetType, _buffer.size());
            resetTracking();
        }
        if (_buffer.size() >= 40)
        {
            resetTracking();
        }
    }
    else if (currentlyBuildingAsciiPacket)
    {
        _buffer.push_back(dataByte);

        if (dataByte == AsciiEscapeChar)
        {
            resetTracking();
        }
        else if (dataByte == AsciiEndChar1)
        {
            asciiEndChar1Found = true;
        }
        else if (asciiEndChar1Found)
        {
            if (dataByte == AsciiEndChar2)
            {
                // We have a possible data packet
                auto p = std::make_unique<uart::protocol::Packet>(_buffer, &sensor);

                if (p->isValid())
                {
                    // We have a valid ascii packet!!!.
                    LOG_TRACE("Valid ascii packet: {}", p->datastr().substr(0, p->getRawDataLength() - 2));
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("Invalid ascii packet: {}", p->datastr());
            }

            resetTracking();
        }

        if (_buffer.size() >= MaximumSizeForAsciiPacket)
        {
            LOG_ERROR("Buffer exceeded the Maximum Ascii Packet Size");
            resetTracking();
        }
    }

    return nullptr;
}

void NAV::sensors::kvh::KvhUartSensor::packetFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData)
{
    LOG_TRACE("called");
    auto* sensor = static_cast<KvhUartSensor*>(userData);

    for (size_t i = 0; i < data.size(); i++, sensor->runningDataIndex++)
    {
        auto packetPointer = sensor->findPacket(data.at(i));

        if (packetPointer != nullptr)
        {
            uart::protocol::Packet packet = *packetPointer;
            dispatchPacket(dispatchPacketUserData, packet, sensor->runningDataIndex, timestamp);
        }
    }
}

uart::protocol::Packet::Type NAV::sensors::kvh::KvhUartSensor::packetTypeFunction(const uart::protocol::Packet& packet)
{
    LOG_TRACE("called");

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
    data_zero = uart::stoh(data_zero, endianness);

    if (data_zero == HEADER_FMT_A || data_zero == HEADER_FMT_B || data_zero == HEADER_FMT_C)
    {
        return uart::protocol::Packet::Type::TYPE_BINARY;
    }

    return uart::protocol::Packet::Type::TYPE_UNKNOWN;
}

bool NAV::sensors::kvh::KvhUartSensor::checksumFunction(const uart::protocol::Packet& packet)
{
    LOG_TRACE("called");

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
        checksumPacket = uart::stoh(checksumPacket, endianness);

        return checksumPacket == checksumCalc;
    }

    LOG_CRITICAL("Can't calculate checksum of packet with unknown type");
    return false;
}

bool NAV::sensors::kvh::KvhUartSensor::isErrorFunction(const uart::protocol::Packet& /* packet */)
{
    LOG_TRACE("called");

    return false;
}

bool NAV::sensors::kvh::KvhUartSensor::isResponseFunction(const uart::protocol::Packet& /* packet */)
{
    LOG_TRACE("called");

    return false;
}