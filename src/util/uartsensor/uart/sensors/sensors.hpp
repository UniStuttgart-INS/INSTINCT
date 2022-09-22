// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file sensors.hpp
/// @brief Extract from the sensor implementation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_UARTSENSOR_LIBRARY

    #include <string>
    #include <vector>

    #include "uart/protocol/packet.hpp"
    #include "uart/util/utilities.hpp"
    #include "uart/xplat/timestamp.hpp"

namespace uart::xplat
{
/// @brief Placeholder class
class IPort
{};

} // namespace uart::xplat

namespace uart::sensors
{
/// \brief Helpful class for working with Uart sensors.
class UartSensor
{
  public:
    /// Packet class
    friend struct protocol::Packet;

    /// Size of the Serial Port read buffer
    static constexpr size_t DefaultReadBufferSize = 4096;

    /// \brief Defines the signature for a method that can receive
    /// notifications of new valid packets found.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerPossiblePacketFoundHandler.
    /// \param[in] packet The packet that was found.
    /// \param[in] runningIndexOfPacketStart The running index of the start of the packet.
    /// \param[in] timestamp The timestamp the packet was found.
    using ValidPacketFoundHandler = void (*)(void* userData, protocol::Packet& packet, size_t runningIndexOfPacketStart, const xplat::TimeStamp& timestamp);

    /// \brief Defines the signature for a method that can handle received data
    ///
    /// \param[in] data The data array
    /// \param[in] timestamp The timestamp the packet was found.
    /// \param[in] dispatchPacket Callback when possible packet was found
    /// \param[in] dispatchPacketUserData Pointer to user data that was initially supplied to the dispatch function
    /// \param[in] userData Pointer to user data that was initially supplied when the callback was registered
    using PacketFinderFunction = void (*)(const std::vector<uint8_t>& data, const xplat::TimeStamp& timestamp, ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData, void* userData);

    /// \brief Defines a callback handler that can received notification when
    /// the UartSensor receives raw data from its port.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerRawDataReceivedHandler.
    /// \param[in] rawData Pointer to the raw data.
    /// \param[in] runningIndex The running index of the received data.
    using RawDataReceivedHandler = void (*)(void* userData, const std::vector<uint8_t>& rawData, size_t runningIndex);

    /// \brief Defines the signature for a method that can receive
    /// notifications of new possible packets found.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerPossiblePacketFoundHandler.
    /// \param[in] possiblePacket The possible packet that was found.
    /// \param[in] packetStartRunningIndex The running index of the start of
    ///     the packet.
    using PossiblePacketFoundHandler = void (*)(void* userData, protocol::Packet& possiblePacket, size_t packetStartRunningIndex);

    /// \brief Defines the signature for a method that can receive
    /// notifications of when a new asynchronous data packet (ASCII or BINARY)
    /// is received.
    ///
    /// This packet will have already had and pertinent error checking
    /// performed and determined to be an asynchronous packet.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerAsyncPacketReceivedHandler.
    /// \param[in] asyncPacket The asynchronous packet received.
    /// \param[in] packetStartRunningIndex The running index of the start of
    ///     the packet.
    using AsyncPacketReceivedHandler = void (*)(void* userData, protocol::Packet& asyncPacket, size_t packetStartRunningIndex);

    /// \brief Defines the signature for a method that can receive
    /// notifications when an error message is received.
    ///
    /// This packet will have already had and pertinent error checking
    /// performed and determined to be an asynchronous packet.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerErrorPacketReceivedHandler.
    /// \param[in] errorPacket The error packet received.
    /// \param[in] packetStartRunningIndex The running index of the start of
    ///     the packet.
    using ErrorPacketReceivedHandler = void (*)(void* userData, protocol::Packet& errorPacket, size_t packetStartRunningIndex);

    /// \brief Defines a callback handler that can be called to check a certain packet property
    ///
    /// \param[in] packet The packet to run the check for
    /// \return Returns true if the check succeeded
    using PacketCheckFunction = bool (*)(const protocol::Packet& packet);

    /// \brief Defines a callback handler that can be called to determine the packet type
    ///
    /// \param[in] packet The packet to run the check for
    /// \return Returns the packet type
    using PacketTypeFunction = protocol::Packet::Type (*)(const protocol::Packet& packet);

    /// \brief The list of baudrates supported by uart sensors.
    static std::vector<uint32_t> supportedBaudrates();

    /// @brief Constructor
    /// @param[in, out] endianness Endianness of the Sensor
    /// @param[in, out] packetFinderFunction Function to Call with the Raw data to find packets in it
    /// @param[in, out] packetFinderUserData UserData needed while finding packets
    /// @param[in, out] packetTypeFunction Function determining the packet type
    /// @param[in, out] isValidFunction Function which checks the checksum
    /// @param[in, out] isErrorFunction Function which checks if the packet is an error message
    /// @param[in, out] isResponseFunction Function which checks if the packet is a response message
    /// @param[in, out] packetHeaderLength Length of the Header in each packet
    UartSensor(Endianness endianness,
               PacketFinderFunction packetFinderFunction,
               void* packetFinderUserData,
               PacketTypeFunction packetTypeFunction,
               PacketCheckFunction isValidFunction,
               PacketCheckFunction isErrorFunction,
               PacketCheckFunction isResponseFunction,
               size_t packetHeaderLength);

    /// @brief Destructor
    ~UartSensor() = default;

    /// @brief Copy constructor
    UartSensor(const UartSensor&) = delete;
    /// @brief Move Constructor
    UartSensor(UartSensor&&) = delete;
    /// @brief Copy assignment operator
    UartSensor& operator=(const UartSensor&) = delete;
    /// @brief Move assignment operator
    UartSensor& operator=(UartSensor&&) = delete;

    /// \brief Returns the baudrate of the serial port connection. Note this
    /// is independent of the sensor's on-board serial baudrate setting.
    ///
    /// \return The connected baudrate.
    uint32_t baudrate();

    /// \brief Returns the name of the port the sensor is connected to.
    ///
    /// \return The port name.
    std::string port();

    /// \brief Indicates if the UartSensor is connected.
    ///
    /// \return <c>true</c> if the UartSensor is connected; otherwise <c>false</c>.
    bool isConnected();

    /// \brief Checks if we are able to send and receive communication with a sensor.
    ///
    /// \return <c>true</c> if we can communicate with the sensor; otherwise <c>false</c>.
    static bool verifySensorConnectivity();

    /// \brief Connects to a uart sensor.
    ///
    /// \param[in] portName The name of the serial port to connect to.
    /// \param[in] baudrate The baudrate to connect at.
    void connect(const std::string& portName, uint32_t baudrate);

    /// \brief Disconnects from the uart sensor.
    void disconnect();

    /// \brief Issues a change baudrate to the uart sensor and then
    /// reconnects the attached serial port at the new baudrate.
    ///
    /// \param[in] baudrate The new sensor baudrate.
    void changeBaudRate(uint32_t baudrate);

    /// \brief Registers a callback method for notification when raw data is
    /// received.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerRawDataReceivedHandler(void* userData, RawDataReceivedHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterRawDataReceivedHandler();

    /// \brief Registers a callback method for notification when a new possible
    /// packet is found.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerPossiblePacketFoundHandler(void* userData, PossiblePacketFoundHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterPossiblePacketFoundHandler();

    /// \brief Registers a callback method for notification when a new
    /// asynchronous data packet is received.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerAsyncPacketReceivedHandler(void* userData, AsyncPacketReceivedHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterAsyncPacketReceivedHandler();

    /// \brief Registers a callback method for notification when an error
    /// packet is received.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerErrorPacketReceivedHandler(void* userData, ErrorPacketReceivedHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterErrorPacketReceivedHandler();

  private:
    struct Impl;
    Impl* _pi;

    const Endianness _endianness;
    const PacketFinderFunction _packetFinderFunction;
    void* _packetFinderUserData;
    const PacketTypeFunction _packetTypeFunction;
    const PacketCheckFunction _isValidFunction;
    const PacketCheckFunction _isErrorFunction;
    const PacketCheckFunction _isResponseFunction;
    const size_t _packetHeaderLength;
};

} // namespace uart::sensors

#endif