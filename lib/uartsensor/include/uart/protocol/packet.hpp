#pragma once

#include <string>
#include <cstring>
#include <vector>

#include "uart/util/nocopy.hpp"
#include "uart/xplat/int.hpp"
#include "uart/xplat/export.hpp"
#include "uart/util/utilities.hpp"

namespace uart::sensors
{
class UartSensor;
} // namespace uart::sensors

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

    /// \brief  Default constructor
    /// \param[in] backReference Reference to the parent UartSensor
    explicit Packet(sensors::UartSensor* backReference);

    /// \brief Creates a new packet based on the provided packet data buffer. A full
    /// packet is expected which contains the deliminators
    ///
    /// \param[in] data Pointer to buffer containing the packet.
    /// \param[in] backReference Reference to the parent UartSensor
    Packet(std::vector<uint8_t> data, sensors::UartSensor* backReference);

    /// \brief Creates a new packet based on the provided string.
    ///
    /// \param[in] packet String containing the packet.
    /// \param[in] backReference Reference to the parent UartSensor
    Packet(const std::string& packet, sensors::UartSensor* backReference);

    /// \brief Default Constructor
    Packet() = default;
    /// \brief Destructor
    ~Packet() = default;

    /// \brief Copy constructor.
    Packet(const Packet& toCopy) = default;
    /// \brief Copy assignment operator
    Packet& operator=(const Packet& from);
    /// \brief Move constructor
    Packet(Packet&&) = default;
    /// \brief Move assignment operator
    Packet& operator=(Packet&&) = default;

    /// \brief Return the raw data
    [[nodiscard]] const std::vector<uint8_t>& getRawData() const;

    /// \brief Returns the raw data length
    [[nodiscard]] size_t getRawDataLength() const;

    /// \brief Returns the encapsulated data as a string.
    ///
    /// \return The packet data.
    [[nodiscard]] std::string datastr() const;

    /// \brief Returns the type of packet.
    ///
    /// \return The type of packet.
    [[nodiscard]] Type type() const;

    /// \brief Performs data integrity check on the data packet.
    ///
    /// This will perform an 8-bit XOR checksum, a CRC16-CCITT CRC, or no
    /// checking depending on the provided data integrity in the packet.
    ///
    /// \return <c>true</c> if the packet passed the data integrity checks;
    ///     otherwise <c>false</c>.
    [[nodiscard]] bool isValid() const;

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

    /// @brief Extracts a int32_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    int32_t extractInt32();

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

    sensors::UartSensor* _backReference{ nullptr };
};

} // namespace uart::protocol