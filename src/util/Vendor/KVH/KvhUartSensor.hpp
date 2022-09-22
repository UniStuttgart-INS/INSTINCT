// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KvhUartSensor.hpp
/// @brief Class to read out KVH Sensors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-07-28

#pragma once

#include <memory>

#include "uart/sensors/sensors.hpp"

namespace NAV::vendor::kvh
{
/// @brief Class to read out KVH Sensors
class KvhUartSensor
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Parent Node
    explicit KvhUartSensor(std::string name);

    /// @brief Default constructor
    KvhUartSensor() = default;
    /// @brief Destructor
    ~KvhUartSensor() = default;
    /// @brief Copy constructor
    KvhUartSensor(const KvhUartSensor&) = delete;
    /// @brief Move constructor
    KvhUartSensor(KvhUartSensor&&) = delete;
    /// @brief Copy assignment operator
    KvhUartSensor& operator=(const KvhUartSensor&) = delete;
    /// @brief Move assignment operator
    KvhUartSensor& operator=(KvhUartSensor&&) = delete;
    /// @brief Arrow operator overload
    uart::sensors::UartSensor* operator->() { return &_sensor; };

    /// @brief Collects data bytes and searches for packages inside of them
    /// @param[in] dataByte The next data byte
    /// @return nullptr if no packet found yet, otherwise a pointer to the packet
    std::unique_ptr<uart::protocol::Packet> findPacket(uint8_t dataByte);

    static constexpr uint32_t HEADER_FMT_A = 0xFE81FF55;     ///< Header Format A
    static constexpr uint32_t HEADER_FMT_B = 0xFE81FF56;     ///< Header Format B
    static constexpr uint32_t HEADER_FMT_C = 0xFE81FF57;     ///< Header Format C
    static constexpr uint32_t HEADER_FMT_XBIT = 0xFE8100AA;  ///< Header Format X Bit
    static constexpr uint32_t HEADER_FMT_XBIT2 = 0xFE8100AB; ///< Header Format X Bit 2

    static constexpr uart::Endianness ENDIANNESS = uart::Endianness::ENDIAN_BIG; ///< Endianess of the sensor

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

    static constexpr size_t PACKET_HEADER_LENGTH = 0; ///< Length of the packet header

    static constexpr uint8_t ASCII_END_CHAR_1 = '\r';    ///< First Ascii End character
    static constexpr uint8_t ASCII_END_CHAR_2 = '\n';    ///< Second Ascii End character
    static constexpr uint8_t ASCII_ESCAPE_CHAR = '\0';   ///< Ascii Escape charater
    static constexpr size_t MAX_SIZE_ASCII_PACKET = 256; ///< Maximum size of a ascii packet before resetting it

    bool _currentlyBuildingAsciiPacket{ false };  ///< Flag if currently a ascii packet is built
    bool _currentlyBuildingBinaryPacket{ false }; ///< Flag if currently a binary packet is built

    bool _asciiEndChar1Found{ false }; ///< Flag if the first ascii end character was found

    /// @brief Possible states in the header building process
    enum TagState
    {
        SM_H1,
        SM_H2,
        SM_H3,
        SM_X3,
        SM_IDLE
    };

    /// Current state of the header building process
    TagState _eState = SM_IDLE;

    /// @brief Possible Header Types
    enum HeaderType
    {
        FMT_A,
        FMT_B,
        FMT_C,
        FMT_XBIT,
        FMT_XBIT2,
        FMT_UNKNOWN
    };

    /// @brief Current packet type determined by the header
    HeaderType _packetType = HeaderType::FMT_UNKNOWN;

    /// @brief Function which finds the header from the provided data
    /// @param[in] ui8Data Byte to construct the header from
    /// @return Header type if found or unkown
    HeaderType bFindImuHeader(uint8_t ui8Data);

    /// Buffer to collect messages till they are complete
    std::vector<uint8_t> _buffer;

    /// Used for correlating raw data with where the packet was found for the end user.
    size_t _runningDataIndex{ 0 };

    /// @brief Resets the current message tracking
    void resetTracking();
};

} // namespace NAV::vendor::kvh
