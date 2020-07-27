#include "UbloxUartSensor.hpp"

#include "UbloxUtilities.hpp"
#include "util/Logger.hpp"

constexpr uint8_t BinarySyncChar1 = 0xB5;
constexpr uint8_t BinarySyncChar2 = 0x62;

void NAV::sensors::UbloxUartSensor::packageFinderFunction(const std::vector<uint8_t>& data, const uart::xplat::TimeStamp& timestamp, uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, uart::sensors::UartSensor* uartSensor)
{
    LOG_TRACE("called");

    uart::protocol::Packet packet{ data, uartSensor };
    dispatchPacket(dispatchPacketUserData, packet, 0, timestamp);
}

uart::protocol::Packet::Type NAV::sensors::UbloxUartSensor::packetTypeFunction(const uart::protocol::Packet& packet)
{
    LOG_TRACE("called");

    if (packet.getRawDataLength() < 1)
    {
        LOG_CRITICAL("Packet does not contain any data.");
    }

    const uint8_t data_zero = packet.getRawData().at(0);

    if (data_zero == '$')
    {
        return uart::protocol::Packet::Type::TYPE_ASCII;
    }
    if (data_zero == BinarySyncChar1)
    {
        if (packet.getRawData().at(1) == BinarySyncChar2)
        {
            return uart::protocol::Packet::Type::TYPE_BINARY;
        }
    }

    return uart::protocol::Packet::Type::TYPE_UNKNOWN;
}

bool NAV::sensors::UbloxUartSensor::checksumFunction(const uart::protocol::Packet& packet)
{
    LOG_TRACE("called");

    if (packet.getRawDataLength() <= 8)
    {
        return false;
    }

    if (packet.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        // First check if we have a checksum at all
        if (packet.getRawData().at(packet.getRawDataLength() - 5) == '*')
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

    LOG_CRITICAL("Can't calculate checksum of package with unknown type");
    return false;
}

bool NAV::sensors::UbloxUartSensor::isErrorFunction(const uart::protocol::Packet& /* packet */)
{
    LOG_TRACE("called");

    return false;
}

bool NAV::sensors::UbloxUartSensor::isResponseFunction(const uart::protocol::Packet& /* packet */)
{
    LOG_TRACE("called");

    return false;
}