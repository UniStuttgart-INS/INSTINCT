#pragma once

#include <string>
#include <cstring>
#include <vector>

#include "uart/util/nocopy.hpp"
#include "uart/xplat/int.hpp"
#include "uart/xplat/export.hpp"
#include "uart/util/utilities.hpp"

namespace uart::protocol
{
/// \brief Structure representing a UART packet received from the sensor.
struct proglib_DLLEXPORT Packet
{
    /// \brief The different types of UART packets.
    enum Type
    {
        TYPE_UNKNOWN, ///< Type is unknown.
        TYPE_BINARY,  ///< Binary packet.
        TYPE_ASCII    ///< ASCII packet.
    };

    /// \brief Defines a callback handler that can be called to check a certain packet property
    ///
    /// \param[in] packet The packet to run the check for
    /// \return Returns true if the check succeeded
    using PacketCheckFunction = bool (*)(const protocol::Packet& packet);

    /// \brief Defines a callback handler that can be called to determine the packet type
    ///
    /// \param[in] packet The packet to run the check for
    /// \return Returns the packet type
    using PacketTypeFunction = Type (*)(const protocol::Packet& packet);

    /// Default constructor
    Packet() = default;

    /// \brief Creates a new packet based on the provided packet data buffer. A full
    /// packet is expected which contains the deliminators
    ///
    /// \param[in] data Pointer to buffer containing the packet.
    explicit Packet(std::vector<uint8_t> data);

    /// \brief Creates a new packet based on the provided string.
    ///
    /// \param[in] packet String containing the packet.
    explicit Packet(const std::string& packet);

    /// \brief Copy constructor.
    ///
    /// \param[in] toCopy The Packet to copy.
    Packet(const Packet& toCopy);

    /// \brief Destructor
    ~Packet() = default;

    /// \brief Move constructor
    Packet(Packet&&) = default;
    /// \brief Move assignment operator
    Packet& operator=(Packet&&) = default;

    /// \brief Return the raw data
    const std::vector<uint8_t>& getRawData();

    /// \brief Returns the raw data length
    [[nodiscard]] size_t getRawDataLength() const;

    /// \brief Assignment operator.
    ///
    /// \param[in] from The packet to assign from.
    /// \return Reference to the newly copied packet.
    Packet& operator=(const Packet& from);

    /// \brief Returns the encapsulated data as a string.
    ///
    /// \return The packet data.
    std::string datastr();

    /// \brief Returns the type of packet.
    ///
    /// \return The type of packet.
    Type type();

    /// \brief Performs data integrity check on the data packet.
    ///
    /// This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no
    /// checking depending on the provided data integrity in the packet.
    ///
    /// \return <c>true</c> if the packet passed the data integrity checks;
    ///     otherwise <c>false</c>.
    bool isValid();

    /// \brief Indicates if the packet is an ASCII error message.
    ///
    /// \return <c>true</c> if the packet is an error message; otherwise
    /// <c>false</c>.
    [[nodiscard]] bool isError() const;

    /// \brief Indicates if the packet is a response to a message sent to the
    /// sensor.
    ///
    /// \return <c>true</c> if the packet is a response message; otherwise
    /// <c>false</c>.
    [[nodiscard]] bool isResponse() const;

    /// \defgroup uartPacketBinaryExtractors UART Binary Data Extractors
    /// \brief This group of methods are useful for extracting data from binary
    /// data packets.
    ///
    /// \{

    /// \brief Extracts a uint8_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    uint8_t extractUint8();

    /// \brief Extracts a int8_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    int8_t extractInt8();

    /// \brief Extracts a uint16_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    uint16_t extractUint16();

    /// \brief Extracts a uint32_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    uint32_t extractUint32();

    /// \brief Extracts a uint64_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    uint64_t extractUint64();

    /// \brief Extracts a float fdata type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    float extractFloat();

    /// \brief Extracts a double ddata type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    double extractDouble();

    /// \}

  private:
    void ensureCanExtract(size_t numOfBytes);

    std::vector<uint8_t> _data;
    size_t _curExtractLoc{ 0 };

    Endianness _endianness{ Endianness::ENDIAN_LITTLE };
    PacketTypeFunction _packetTypeFunction{ nullptr };
    PacketCheckFunction _checksumFunction{ nullptr };
    PacketCheckFunction _isErrorFunction{ nullptr };
    PacketCheckFunction _isResponseFunction{ nullptr };
    size_t _packageHeaderLength{};
    size_t _packageEndLength{};
};

} // namespace uart::protocol