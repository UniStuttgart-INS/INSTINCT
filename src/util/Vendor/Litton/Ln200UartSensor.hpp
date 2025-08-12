// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Ln200UartSensor.hpp
/// @brief Class to read out LN-200 Sensors
/// @author M. Senger (st166640@stud.uni-stuttgart.de)
/// @date 2025-03-18

#pragma once

#include <memory>
#include "util/Container/ScrollingBuffer.hpp"

#include "uart/sensors/sensors.hpp"

namespace NAV::vendor::ln
{
/// @brief Class to read out LN-200 Sensors
/// @details The class is used to read out the LN-200 IMU via UART with 2000
/// @note The UART interface is configured to 2Mbit/s, 8N1 set by the IMU Uart interface microcontroller.
///       The following commands are needed to set up the UART interface:
///        sudo usermod -a -G dialout "username"
///        sudo dmesg | grep -i tty -> find connected device
///        stty -F /dev/"device" -raw 2000000 cs8 -cstopb -parenb
///
/// Message content mapping:
/// - Word Index 0: X_VEL       - X Delta Velocity
/// - Word Index 1: Y_VEL       - Y Delta Velocity
/// - Word Index 2: Z_VEL       - Z Delta Velocity
/// - Word Index 3: X_ANG       - X Delta Angle
/// - Word Index 4: Y_ANG       - Y Delta Angle
/// - Word Index 5: Z_ANG       - Z Delta Angle
/// - Word Index 6: STATUS      - IMU Status Summary Word
/// - Word Index 7: MUX_ID      - Mode Bit/MUX ID
/// - Word Index 8: MUX_DATA    - Mux Data Word
/// - Word Index 9: X_RAW_COUNT - X Raw Gyro Counts
/// - Word Index 10: Y_RAW_COUNT - Y Raw Gyro Counts
/// - Word Index 11: Z_RAW_COUNT - Z Raw Gyro Counts
/// - Word Index 12: INTERNAL_CHECK_SUM - Internal Check Sum
/// - Word Index 13: EXTERNAL_CHECK_SUM - External Check Sum
class Ln200UartSensor
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Parent Node
    explicit Ln200UartSensor(std::string name);

    /// @brief Default constructor
    Ln200UartSensor() = default;
    /// @brief Destructor
    ~Ln200UartSensor() = default;
    /// @brief Copy constructor
    Ln200UartSensor(const Ln200UartSensor&) = delete;
    /// @brief Move constructor
    Ln200UartSensor(Ln200UartSensor&&) = delete;
    /// @brief Copy assignment operator
    Ln200UartSensor& operator=(const Ln200UartSensor&) = delete;
    /// @brief Move assignment operator
    Ln200UartSensor& operator=(Ln200UartSensor&&) = delete;
    /// @brief Arrow operator overload
    uart::sensors::UartSensor* operator->() { return &_sensor; };

    /// @brief Collects data bytes and searches for packages inside of them
    /// @param[in] dataByte The next data byte
    /// @return nullptr if no packet found yet, otherwise a pointer to the packet
    std::unique_ptr<uart::protocol::Packet> findPacket(uint8_t dataByte);

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
    static constexpr size_t PACKET_HEADER_LENGTH = 0;                               ///< Length of the packet header
    static constexpr size_t MAX_SIZE_ASCII_PACKET = 256;                            ///< Maximum size of a ascii packet before resetting it

    /// Operational states of the LN-200 UART sensor node.
    enum class State : uint8_t
    {
        Idle,     ///< Sensor is idle.
        Receiving ///< Sensor is receiving data.
    };

    /// Internal state of the sensor.
    State _state = State::Idle;

    /// Buffer to accumulate bits of the current frame.
    ScrollingBuffer<bool> _bitBuffer;

    /// Sliding window buffer to detect flags.
    ScrollingBuffer<bool> _flagWindow;

    /// Flag to detect the start of a frame.
    bool _wasIdle = false;

    /// Counter for bit insertion.
    int _consecutiveOnes = 0;

    /// Used for correlating raw data with where the packet was found for the end user.
    size_t _runningDataIndex{ 0 };
};

} // namespace NAV::vendor::ln
