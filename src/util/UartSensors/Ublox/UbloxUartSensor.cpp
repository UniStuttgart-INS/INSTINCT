#include "UbloxUartSensor.hpp"

#include "UbloxUtilities.hpp"
#include "util/Logger.hpp"

NAV::sensors::ublox::UbloxUartSensor::UbloxUartSensor(std::string name)
    : _name(std::move(name)), _buffer(uart::sensors::UartSensor::DefaultReadBufferSize)
{
    resetTracking();
}

void NAV::sensors::ublox::UbloxUartSensor::resetTracking()
{
    _currentlyBuildingBinaryPacket = false;
    _currentlyBuildingAsciiPacket = false;

    _asciiEndChar1Found = false;
    _binarySyncChar2Found = false;
    _binaryMsgClassFound = false;
    _binaryMsgIdFound = false;
    _binaryPayloadLength1Found = false;
    _binaryPayloadLength2Found = false;

#if (defined(__GNUC__) || defined(__GNUG__)) && !defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
    _binaryMsgClass = 0;
    _binaryMsgId = 0;
    _binaryPayloadLength = 0;
#if (defined(__GNUC__) || defined(__GNUG__)) && !defined(__clang__)
    #pragma GCC diagnostic pop
#endif

    _buffer.resize(0);
    _numOfBytesRemainingForCompletePacket = 0;
}

std::unique_ptr<uart::protocol::Packet> NAV::sensors::ublox::UbloxUartSensor::findPacket(uint8_t dataByte)
{
    if (_buffer.size() == _buffer.capacity())
    {
        // Buffer is full
        resetTracking();
        LOG_ERROR("{}: Discarding current packet, because buffer is full.", _name);
    }

    if (!_currentlyBuildingAsciiPacket && !_currentlyBuildingBinaryPacket)
    {
        // This byte must be the start char
        if (dataByte == BINARY_SYNC_CHAR_1)
        {
            resetTracking();
            _currentlyBuildingBinaryPacket = true;
            _buffer.push_back(dataByte);
        }
        else if (dataByte == ASCII_START_CHAR)
        {
            resetTracking();
            _currentlyBuildingAsciiPacket = true;
            _buffer.push_back(dataByte);
        }
    }
    else if (_currentlyBuildingBinaryPacket)
    {
        _buffer.push_back(dataByte);

        if (!_binarySyncChar2Found)
        {
            // This byte must be the second sync char
            if (dataByte == BINARY_SYNC_CHAR_2)
            {
                _binarySyncChar2Found = true;
            }
            else
            {
                resetTracking();
            }
        }
        else if (!_binaryMsgClassFound)
        {
            // This byte must be the message class
            _binaryMsgClassFound = true;
            _binaryMsgClass = dataByte;
        }
        else if (!_binaryMsgIdFound)
        {
            // This byte must be the message id
            _binaryMsgIdFound = true;
            _binaryMsgId = dataByte;
        }
        else if (!_binaryPayloadLength1Found)
        {
            _binaryPayloadLength1Found = true;
            _binaryPayloadLength = static_cast<uint16_t>(dataByte);
        }
        else if (!_binaryPayloadLength2Found)
        {
            _binaryPayloadLength2Found = true;
            _binaryPayloadLength |= static_cast<uint16_t>(static_cast<uint16_t>(dataByte) << 8U);
            _binaryPayloadLength = uart::stoh(_binaryPayloadLength, ENDIANNESS);
            _numOfBytesRemainingForCompletePacket = _binaryPayloadLength + 2U;
            LOG_DATA("{}: Binary packet: Class={:0x}, Id={:0x}, payload length={}", _name, _binaryMsgClass, _binaryMsgId, _binaryPayloadLength);
        }
        else
        {
            // We are currently collecting data for our packet.
            _numOfBytesRemainingForCompletePacket--;

            if (_numOfBytesRemainingForCompletePacket == 0)
            {
                // We have a possible binary packet!
                auto p = std::make_unique<uart::protocol::Packet>(_buffer, &_sensor);

                if (p->isValid())
                {
                    // We have a valid binary packet!!!.
                    resetTracking();
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("{}: Invalid binary packet: Class={:0x}, Id={:0x}, payload length={}", _name, _binaryMsgClass, _binaryMsgId, _binaryPayloadLength);
                resetTracking();
            }
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
                    return p;
                }
                // Invalid packet!
                LOG_ERROR("{}: Invalid ascii packet: {}", _name, p->datastr());
            }

            resetTracking();
        }
    }

    return nullptr;
}

void NAV::sensors::ublox::UbloxUartSensor::packetFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData)
{
    auto* sensor = static_cast<UbloxUartSensor*>(userData);

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
    if (packet.getRawData().at(0) == BINARY_SYNC_CHAR_1)
    {
        if (packet.getRawData().at(1) == BINARY_SYNC_CHAR_2)
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