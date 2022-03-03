/// @file UbloxObs.hpp
/// @brief ublox Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-19

#pragma once

#include "NodeData/InsObs.hpp"

#include <variant>

#include "util/Eigen.hpp"
#include "uart/protocol/packet.hpp"

#include "util/UartSensors/Ublox/UbloxTypes.hpp"

namespace NAV
{
/// ublox Observation Class
class UbloxObs : public InsObs
{
  public:
    /// @brief Constructor
    ///
    /// @param[in] packet The packet to copy into the raw data
    explicit UbloxObs(uart::protocol::Packet& packet)
        : raw(packet) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "UbloxObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// Ubx Message Class (NONE if NMEA message)
    sensors::ublox::UbxClass msgClass = sensors::ublox::UbxClass::UBX_CLASS_NONE;
    /// Ubx Message ID
    uint8_t msgId = 0;
    /// Payload length in bytes
    uint16_t payloadLength = 0;

    /// Complete message raw binary data including header and checksum (UBX) or raw string (NMEA)
    uart::protocol::Packet raw;

    /// Decoded data
    std::variant<
        // ACK: Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
        sensors::ublox::UbxAckAck, sensors::ublox::UbxAckNak,
        // CFG: Configuration Input Messages: Configure the receiver
        // ESF: External Sensor Fusion Messages: External Sensor Measurements and Status Information
        sensors::ublox::UbxEsfIns,
        sensors::ublox::UbxEsfMeas,
        sensors::ublox::UbxEsfRaw,
        sensors::ublox::UbxEsfStatus,
        // HNR: High Rate Navigation Results Messages: High rate time, position, speed, heading
        // INF: Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        // LOG: Logging Messages: Log creation, deletion, info and retrieval
        // MGA: Multiple GNSS Assistance Messages: Assistance data for various GNSS
        // MON:Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
        // NAV: Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
        sensors::ublox::UbxNavAtt,
        sensors::ublox::UbxNavPosecef,
        sensors::ublox::UbxNavPosllh,
        sensors::ublox::UbxNavVelned,
        // RXM: Receiver Manager Messages: Satellite Status, RTC Status
        sensors::ublox::UbxRxmRawx,
        sensors::ublox::UbxRxmSfrbx
        // SEC: Security Feature Messages
        // TIM: Timing Messages: Time Pulse Output, Time Mark Results
        // UPD: Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
        >
        data{};
};

} // namespace NAV
