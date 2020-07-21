#pragma once

#include "uart/util/nocopy.hpp"
#include "uart/xplat/timestamp.hpp"
#include "uart/protocol/packet.hpp"
#include "uart/xplat/export.hpp"

#include <array>
#include <vector>

namespace uart::protocol
{
/// \brief Helps with management of communication with a sensor using the UART
/// protocol.
///
/// Internally, the PacketFinder keeps track of a running data index which
/// keeps a running count of the bytes that are processed by the class. This is
/// useful for users who wish to keep track of where packets where found in the
/// incoming raw data stream. When the PacketFinder receives its first byte
/// from the user, this is given the index of 0 for the running index and
/// incremented for each byte received.
class proglib_DLLEXPORT PacketFinder : private util::NoCopy
{
  public:
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
    using ValidPacketFoundHandler = void (*)(void* userData, Packet& packet, size_t runningIndexOfPacketStart, const xplat::TimeStamp& timestamp);

    /// \brief Defines the signature for a method that can handle received data
    ///
    /// \param[in] userData Pointer to user data that was initially supplied
    ///     when the callback was registered via registerProcessReceivedDataHandler.
    /// \param[in] data The data array
    /// \param[in] timestamp The timestamp the packet was found.
    /// \param[in] dispatchPacket Callback when possible package was found
    /// \param[in] dispatchPacketUserData Pointer to user data that was initially supplied to the dispatch function
    using ProcessReceivedDataHandler = void (*)(const std::vector<uint8_t>& data, const xplat::TimeStamp& timestamp, ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData);

    PacketFinder() = default;                              ///< Default constructor
    ~PacketFinder() = default;                             ///< Destructor
    PacketFinder(const PacketFinder&) = delete;            ///< Copy constructor
    PacketFinder(PacketFinder&&) = delete;                 ///< Move constructor
    PacketFinder& operator=(const PacketFinder&) = delete; ///< Copy assignment operator
    PacketFinder& operator=(PacketFinder&&) = delete;      ///< Move assignment operator

    /// \brief Adds new data to the internal buffers and processes the received
    /// data to determine if any new received packets are available.
    ///
    /// \param[in] data The data buffer containing the received data.
    /// \param[in] length The length of the data array
    /// \param[in] timestamp The time when the data was received.
    void processReceivedData(const std::vector<uint8_t>& data, const xplat::TimeStamp& timestamp);

    /// \brief Registers a callback method for dealing with new received data
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the callback method.
    /// \param[in] handler The callback method.
    void registerProcessReceivedDataHandler(void* userData, ProcessReceivedDataHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterProcessReceivedDataHandler();

    /// \brief Registers a callback method for notification when a new possible
    /// packet is found.
    ///
    /// \param[in] userData Pointer to user data, which will be provided to the
    ///     callback method.
    /// \param[in] handler The callback method.
    void registerPossiblePacketFoundHandler(void* userData, ValidPacketFoundHandler handler);

    /// \brief Unregisters the registered callback method.
    void unregisterPossiblePacketFoundHandler();

  private:
    void* _possiblePacketFoundUserData{ nullptr };
    ValidPacketFoundHandler _possiblePacketFoundHandler{ nullptr };

    void* _processReceivedDataUserData{ nullptr };
    ProcessReceivedDataHandler _processReceivedDataHandler{ nullptr };
};

} // namespace uart::protocol