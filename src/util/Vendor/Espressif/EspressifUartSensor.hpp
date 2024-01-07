// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EspressifUartSensor.hpp
/// @brief Class to read out Espressif Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-22

#pragma once

#include <memory>

#include "uart/sensors/sensors.hpp"

namespace NAV::vendor::espressif
{
/// @brief Class to read out Espressif Sensors
class EspressifUartSensor
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Parent Node
    explicit EspressifUartSensor(std::string name);

    /// @brief Default constructor
    EspressifUartSensor() = default;
    /// @brief Destructor
    ~EspressifUartSensor() = default;
    /// @brief Copy constructor
    EspressifUartSensor(const EspressifUartSensor&) = delete;
    /// @brief Move constructor
    EspressifUartSensor(EspressifUartSensor&&) = delete;
    /// @brief Copy assignment operator
    EspressifUartSensor& operator=(const EspressifUartSensor&) = delete;
    /// @brief Move assignment operator
    EspressifUartSensor& operator=(EspressifUartSensor&&) = delete;
    /// @brief Arrow operator overload
    uart::sensors::UartSensor* operator->() { return &_sensor; };

    /// @brief Collects data bytes and searches for packages inside of them
    /// @param[in] dataByte The next data byte
    /// @return nullptr if no packet found yet, otherwise a pointer to the packet
    std::unique_ptr<uart::protocol::Packet> findPacket(uint8_t dataByte);

    static constexpr uint8_t BINARY_SYNC_CHAR_1 = 0xB5; ///< µ - First sync character which begins a new binary message
    static constexpr uint8_t BINARY_SYNC_CHAR_2 = 0x62; ///< b - Second sync character which begins a new binary message

  private:
    /// Name of the Parent Node
    const std::string _name;

    /// UartSensor object which handles the UART interface
    uart::sensors::UartSensor _sensor{ ENDIANNESS,
                                       packetFinderFunction,
                                       this,
                                       packetTypeFunction,
                                       checksumFunction,
                                       isErrorFunction,
                                       isResponseFunction,
                                       PACKET_HEADER_LENGTH };

    /// @brief Function which is called to find packets in the provided data buffer
    /// @param[in] data Raw data buffer which has potential packets inside
    /// @param[in] timestamp Timestamp then the data in the buffer was received
    /// @param[in] dispatchPacket Function to call when a complete packet was found
    /// @param[in] dispatchPacketUserData User data to forward to the dispatchPacket function
    /// @param[in] userData User data provided when regisering this function. Should contain the sensor object
    static void packetFinderFunction(const std::vector<uint8_t>& data,
                                     const uart::xplat::TimeStamp& timestamp,
                                     uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData,
                                     void* userData);

    /// @brief Function which is called to determine the packet type (ascii/binary)
    /// @param[in] packet Packet to check the type of
    /// @return The type of the packet
    static uart::protocol::Packet::Type packetTypeFunction(const uart::protocol::Packet& packet);

    /// @brief Function which is called to verify packet integrity
    /// @param[in] packet Packet to calculate the checksum for
    /// @return True if the packet is fault free
    static bool checksumFunction(const uart::protocol::Packet& packet);

    /// @brief Function which determines, if the packet is an Error Packet
    /// @param[in] packet The packet to check
    static bool isErrorFunction(const uart::protocol::Packet& packet);

    /// @brief Function which determines, if the packet is a Response
    /// @param[in] packet The packet to check
    static bool isResponseFunction(const uart::protocol::Packet& packet);

    static constexpr uart::Endianness ENDIANNESS = uart::Endianness::ENDIAN_LITTLE; ///< Endianess of the sensor
    static constexpr size_t PACKET_HEADER_LENGTH = 2;                               ///< Length of the header of each packet

    bool _currentlyBuildingBinaryPacket{ false }; ///< Flag if currently a binary packet is built

    bool _binarySyncChar2Found{ false };      ///< Flag if the second binary end character was found
    bool _binaryPayloadLength1Found{ false }; ///< Flag if the first byte of the payload length was found
    bool _binaryPayloadLength2Found{ false }; ///< Flag if the second byte of the payload length was found

    /// Payload length of the current packet
    uint16_t _binaryPayloadLength{ 0 };

    /// Buffer to collect messages till they are complete
    std::vector<uint8_t> _buffer;

    /// Used for correlating raw data with where the packet was found for the end user.
    size_t _runningDataIndex{ 0 };
    /// Amount of bytes remaining for a complete packet
    size_t _numOfBytesRemainingForCompletePacket{ 0 };

    /// @brief Resets the current message tracking
    void resetTracking();
};

} // namespace NAV::vendor::espressif
