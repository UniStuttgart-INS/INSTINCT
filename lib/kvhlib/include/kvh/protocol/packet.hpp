#pragma once

#include <string>
#include <cstring>

#include "kvh/util/nocopy.hpp"
#include "kvh/xplat/int.hpp"
#include "kvh/xplat/export.hpp"

namespace kvh::protocol::uart
{
/// \brief Structure representing a UART packet received from the sensor.
struct kvh_proglib_DLLEXPORT Packet
{
    /// \brief The different types of UART packets.
    enum Type
    {
        TYPE_UNKNOWN,      ///< Type is unknown.
        TYPE_ASCII,        ///< ASCII packet.
        TYPE_BINARY_FMT_A, ///< Binary packet.
        TYPE_BINARY_FMT_B, ///< Binary packet.
        TYPE_BINARY_FMT_C, ///< Binary packet.
        TYPE_BINARY_TEST1, ///< Binary packet.
        TYPE_BINARY_TEST2  ///< Binary packet.
    };

    static constexpr uint32_t HEADER_FMT_A = 0xFE81FF55;
    static constexpr uint32_t HEADER_FMT_B = 0xFE81FF56;
    static constexpr uint32_t HEADER_FMT_C = 0xFE81FF57;
    static constexpr uint32_t HEADER_FMT_XBIT = 0xFE8100AA;
    static constexpr uint32_t HEADER_FMT_XBIT2 = 0xFE8100AB;

    Packet();

    /// \brief Creates a new packet based on the provided packet data buffer. A full
    /// packet is expected which contains the deliminators
    ///
    /// \param[in] packet Pointer to buffer containing the packet.
    /// \param[in] length The number of bytes in the packet.
    Packet(unsigned char const* packet, size_t length);

    explicit Packet(std::string packet);

    /// \brief Copy constructor.
    ///
    /// \param[in] toCopy The Packet to copy.
    Packet(const Packet& toCopy);

    ~Packet();

    /// \brief Return the raw data
    unsigned char* getRawData();

    /// \brief Returns the raw data length
    size_t getRawDataLength();

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
    bool isError();

    /// \brief Indicates if the packet is a response to a message sent to the
    /// sensor.
    ///
    /// \return <c>true</c> if the packet is a response message; otherwise
    /// <c>false</c>.
    bool isResponse();

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

    bool _isPacketDataMine;
    size_t _length;
    unsigned char* _data;
    size_t _curExtractLoc;
};

} // namespace kvh::protocol::uart