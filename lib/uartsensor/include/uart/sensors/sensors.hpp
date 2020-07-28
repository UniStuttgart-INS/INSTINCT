#pragma once

#include <string>
#include <vector>

#include "uart/util/nocopy.hpp"
#include "uart/xplat/export.hpp"
#include "uart/xplat/int.hpp"
#include "uart/protocol/packetfinder.hpp"
#include "uart/protocol/packet.hpp"
#include "uart/xplat/timestamp.hpp"

namespace uart::xplat
{
class IPort;
} // namespace uart::xplat

namespace uart::sensors
{
/// \brief Helpful class for working with Uart sensors.
class proglib_DLLEXPORT UartSensor : private util::NoCopy
{
  public:
    friend protocol::Packet;

    /// Size of the Serial Port read buffer
    static constexpr size_t DefaultReadBufferSize = 1024;

    /// \brief Defines the signature for a method that can receive
    /// notifications of new valid packets found.
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerPossiblePacketFoundHandler.
    /// \param[in] possiblePacket The possible packet that was found.
    /// \param[in] packetStartRunningIndex The running index of the start of the packet.
    /// \param[in] timestamp The timestamp the packet was found.
    using ValidPacketFoundHandler = void (*)(void* userData, protocol::Packet& packet, size_t runningIndexOfPacketStart, const xplat::TimeStamp& timestamp);

    /// \brief Defines the signature for a method that can handle received data
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerProcessReceivedDataHandler.
    /// \param[in] data The data array
    /// \param[in] timestamp The timestamp the packet was found.
    /// \param[in] dispatchPacket Callback when possible packet was found
    /// \param[in] dispatchPacketUserData Pointer to user data that was initially supplied to the dispatch function
    /// \param[in] User Data Provided when registering the function
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

    UartSensor(Endianness endianness,
               PacketFinderFunction packetFinderFunction,
               void* packetFinderUserData,
               PacketTypeFunction packetTypeFunction,
               PacketCheckFunction isValidFunction,
               PacketCheckFunction isErrorFunction,
               PacketCheckFunction isResponseFunction,
               size_t packetHeaderLength);

    ~UartSensor();

    UartSensor(const UartSensor&) = delete;            ///< Copy constructor
    UartSensor(UartSensor&&) = delete;                 ///< Move constructor
    UartSensor& operator=(const UartSensor&) = delete; ///< Copy assignment operator
    UartSensor& operator=(UartSensor&&) = delete;      ///< Move assignment operator

    /// \brief Returns the baudrate of the serial port connection. Note this
    /// is independent of the sensor's on-board serial baudrate setting.
    ///
    /// \return The connected baudrate.
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

    /// \brief Gets the amount of time in milliseconds to wait for a response
    /// during read/writes with the sensor.
    ///
    /// \return The response timeout in milliseconds.
    uint16_t responseTimeoutMs();

    /// \brief Sets the amount of time in milliseconds to wait for a response
    /// during read/writes with the sensor.
    ///
    /// \param[in] timeout The number of milliseconds for response timeouts.
    void setResponseTimeoutMs(uint16_t timeout);

    /// \brief The delay in milliseconds between retransmitting commands.
    ///
    /// \return The retransmit delay in milliseconds.
    uint16_t retransmitDelayMs();

    /// \brief Sets the delay in milliseconds between command retransmits.
    ///
    /// \param[in] delay The retransmit delay in milliseconds.
    void setRetransmitDelayMs(uint16_t delay);

    /// \brief Checks if we are able to send and receive communication with a sensor.
    ///
    /// \return <c>true</c> if we can communicate with the sensor; otherwise <c>false</c>.
    static bool verifySensorConnectivity();

    /// \brief Connects to a uart sensor.
    ///
    /// \param[in] portName The name of the serial port to connect to.
    /// \param[in] baudrate The baudrate to connect at.
    void connect(const std::string& portName, uint32_t baudrate);

    /// \brief Allows connecting to a uart sensor over a \ref vn::common::ISimplePort.
    ///
    /// The caller is responsible for properly destroying the
    /// \ref vn::common::ISimplePort object when this method is used. Also, if
    /// the provided \ref vn::common::ISimplePort is already open, then when
    /// the method \ref disconnect is called, \ref UartSensor will not attempt to
    /// close the port. However, if the \ref vn::common::ISimplePort is not
    /// open, then any subsequent calls to \ref disconnect will close the port.
    ///
    /// \param[in] simplePort An \ref vn::common::ISimplePort. May be either
    ///     currently open or closed.
    void connect(xplat::IPort* port);

    /// \brief Disconnects from the uart sensor.
    void disconnect();

    /// \brief Sends the provided command and returns the response from the sensor.
    ///
    /// If the command does not have an asterisk '*', then a checksum will be performed
    /// and appended based on the current error detection mode. Also, if the line-ending
    /// \\r\\n is not present, these will be added also.
    ///
    /// \param[in] toSend The command to send to the sensor.
    /// \return The response received from the sensor.
    std::string transaction(const std::string& toSend);

    /// \brief Writes a raw data string to the sensor, normally appending an
    /// appropriate error detection checksum.
    ///
    /// If waitForReply is <c>true</c>, then the method will wait for a
    /// response and return the received response. Otherwise, if <c>false</c>,
    /// the method will send the data and return immediately with an empty
    /// string.
    ///
    /// \param[in] toSend The data to send. The method will automatically
    ///     append a checksum/CRC if none is provided.
    /// \param[in] waitForReply Indicates if the method should wait for a
    ///     response and return the received response.
    /// \return The received response if waitForReply is <c>true</c>; otherwise
    ///     this will be an empty string.
    std::string send(const std::string& toSend, bool waitForReply = true);

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
