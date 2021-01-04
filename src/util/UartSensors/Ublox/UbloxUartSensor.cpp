#include "UbloxUartSensor.hpp"

#include "UbloxUtilities.hpp"
#include "util/Logger.hpp"

NAV::sensors::ublox::UbloxUartSensor::UbloxUartSensor(std::string name)
    : name(std::move(name)), _buffer(uart::sensors::UartSensor::DefaultReadBufferSize)
{
    resetTracking();
}

void NAV::sensors::ublox::UbloxUartSensor::resetTracking()
{
    currentlyBuildingBinaryPacket = false;
    currentlyBuildingAsciiPacket = false;

    asciiEndChar1Found = false;
    binarySyncChar2Found = false;
    binaryMsgClassFound = false;
    binaryMsgIdFound = false;
    binaryPayloadLength1Found = false;
    binaryPayloadLength2Found = false;

    binaryMsgClass = 0;
    binaryMsgId = 0;
    binaryPayloadLength = 0;

    _buffer.resize(0);
    numOfBytesRemainingForCompletePacket = 0;
}

std::unique_ptr<uart::protocol::Packet> NAV::sensors::ublox::UbloxUartSensor::findPacket(uint8_t dataByte)
{
    if (_buffer.size() == _buffer.capacity())
    {
        // Buffer is full
        resetTracking();
        LOG_ERROR("{}: Discarding current packet, because buffer is full.", name);
    }

    if (!currentlyBuildingAsciiPacket && !currentlyBuildingBinaryPacket)
    {
        // This byte must be the start char
        if (dataByte == BinarySyncChar1)
        {
            resetTracking();
            currentlyBuildingBinaryPacket = true;
            _buffer.push_back(dataByte);
        }
        else if (dataByte == AsciiStartChar)
        {
            resetTracking();
            currentlyBuildingAsciiPacket = true;
            _buffer.push_back(dataByte);
        }
    }
    else if (currentlyBuildingBinaryPacket)
    {
        _buffer.push_back(dataByte);

        if (!binarySyncChar2Found)
        {
            // This byte must be the second sync char
            if (dataByte == BinarySyncChar2)
            {
                binarySyncChar2Found = true;
            }
            else
            {
                resetTracking();
            }
        }
        else if (!binaryMsgClassFound)
        {
            // This byte must be the message class
            binaryMsgClassFound = true;
            binaryMsgClass = dataByte;
        }
        else if (!binaryMsgIdFound)
        {
            // This byte must be the message id
            binaryMsgIdFound = true;
            binaryMsgId = dataByte;
        }
        else if (!binaryPayloadLength1Found)
        {
            binaryPayloadLength1Found = true;
            binaryPayloadLength = static_cast<uint16_t>(dataByte);
        }
        else if (!binaryPayloadLength2Found)
        {
            binaryPayloadLength2Found = true;
            binaryPayloadLength |= static_cast<uint16_t>(static_cast<uint16_t>(dataByte) << 8U);
            binaryPayloadLength = uart::stoh(binaryPayloadLength, endianness);
            numOfBytesRemainingForCompletePacket = binaryPayloadLength + 2U;
            LOG_DATA("{}: Binary packet: Class={:0x}, Id={:0x}, payload length={}", name, binaryMsgClass, binaryMsgId, binaryPayloadLength);
        }
        else
        {
            // We are currently collecting data for our packet.
            numOfBytesRemainingForCompletePacket--;

            if (numOfBytesRemainingForCompletePacket == 0)
            {
                // We have a possible binary packet!
                auto p = std::make_unique<uart::protocol::Packet>(_buffer, &sensor);

                if (p->isValid())
                {
                    // We have a valid binary packet!!!.
                    resetTracking();
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("{}: Invalid binary packet: Class={:0x}, Id={:0x}, payload length={}", name, binaryMsgClass, binaryMsgId, binaryPayloadLength);
                resetTracking();
            }
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
                    LOG_DATA("{}: Valid ascii packet: {}", name, p->datastr().substr(0, p->getRawDataLength() - 2));
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("{}: Invalid ascii packet: {}", name, p->datastr());
            }

            resetTracking();
        }
    }

    return nullptr;
}

void NAV::sensors::ublox::UbloxUartSensor::packetFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData)
{
    auto* sensor = static_cast<UbloxUartSensor*>(userData);

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

uart::protocol::Packet::Type NAV::sensors::ublox::UbloxUartSensor::packetTypeFunction(const uart::protocol::Packet& packet)
{
    if (packet.getRawDataLength() < 1)
    {
        LOG_CRITICAL("Packet does not contain any data.");
    }

    if (packet.getRawData().at(0) == '$')
    {
        return uart::protocol::Packet::Type::TYPE_ASCII;
    }
    if (packet.getRawData().at(0) == BinarySyncChar1)
    {
        if (packet.getRawData().at(1) == BinarySyncChar2)
        {
            return uart::protocol::Packet::Type::TYPE_BINARY;
        }
    }

    return uart::protocol::Packet::Type::TYPE_UNKNOWN;
}

bool NAV::sensors::ublox::UbloxUartSensor::checksumFunction(const uart::protocol::Packet& packet)
{
    if (packet.getRawDataLength() <= 8)
    {
        return false;
    }

    if (packet.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        // First check if we have a checksum at all
        if (packet.getRawData().at(packet.getRawDataLength() - 5) != '*')
        {
            return false;
        }

        // Return true, if a wildcard checksum is present
        if (packet.getRawData().at(packet.getRawDataLength() - 3) == 'X'
            && packet.getRawData().at(packet.getRawDataLength() - 4) == 'X')
        {
            return true;
        }

        uint8_t checksumHex = ublox::checksumNMEA(packet.getRawData());
        std::array<uint8_t, 2> checksumRecv = { packet.getRawData().at(packet.getRawDataLength() - 4),
                                                packet.getRawData().at(packet.getRawDataLength() - 3) };
        return uart::to_uint8_from_hexstr(reinterpret_cast<char*>(checksumRecv.data())) == checksumHex;
    }

    if (packet.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        std::pair<uint8_t, uint8_t> checksum = ublox::checksumUBX(packet.getRawData());

        return packet.getRawData().at(packet.getRawDataLength() - 2) == checksum.first
               && packet.getRawData().at(packet.getRawDataLength() - 1) == checksum.second;
    }

    LOG_CRITICAL("Can't calculate checksum of packet with unknown type");
    return false;
}

bool NAV::sensors::ublox::UbloxUartSensor::isErrorFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}

bool NAV::sensors::ublox::UbloxUartSensor::isResponseFunction([[maybe_unused]] const uart::protocol::Packet& packet)
{
    return false;
}