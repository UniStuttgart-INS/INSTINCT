/**
 * @file UbloxPacket.hpp
 * @brief Ublox Packet containing binary data
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-19
 */

#pragma once

#include <string>
#include <cstring>
#include <vector>
#include <cstdint>

namespace NAV::ublox
{
class UbloxPacket
{
  public:
    /// @brief The different types of UART packets.
    enum Type
    {
        TYPE_UNKNOWN, ///< Type is unknown.
        TYPE_BINARY,  ///< Binary packet.
        TYPE_ASCII    ///< ASCII packet.
    };

    /// Default Constructor
    UbloxPacket() = default;

    /// @brief Creates a new packet based on the provided packet data buffer. A full
    /// packet is expected which contains the deliminators
    ///
    /// @param[in] packet Pointer to buffer containing the packet.
    /// @param[in] length The number of bytes in the packet.
    UbloxPacket(unsigned char const* packet, size_t length);

    /**
     * @brief Fill the data vector with the provided packet
     * 
     * @param[in] packet Pointer to buffer containing the packet
     * @param[in] length The number of bytes in the packet
     */
    void setData(unsigned char const* packet, size_t length);

    /// @brief Return the raw data
    [[nodiscard]] const unsigned char* getRawData() const;

    /// @brief Returns the raw data length
    [[nodiscard]] size_t getRawDataLength() const;

    /// @brief Returns the encapsulated data as a string.
    ///
    /// @return The packet data.
    std::string datastr();

    /// @brief Returns the type of packet.
    ///
    /// @return The type of packet.
    [[nodiscard]] Type type() const;

    /// @defgroup uartPacketBinaryExtractors UART Binary Data Extractors
    /// @brief This group of methods are useful for extracting data from binary
    /// data packets.
    ///
    /// @{

    /// @brief Extracts a uint8_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    uint8_t extractUint8();

    /// @brief Extracts a int8_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    int8_t extractInt8();

    /// @brief Extracts a uint16_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    uint16_t extractUint16();

    /// @brief Extracts a uint32_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    uint32_t extractUint32();

    /// @brief Extracts a int32_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    int32_t extractInt32();

    /// @brief Extracts a uint64_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    uint64_t extractUint64();

    /// @brief Extracts a float fdata type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    float extractFloat();

    /// @brief Extracts a double ddata type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// @return The extracted value.
    double extractDouble();

#define U1(p) (*((unsigned char*)(p)))
#define I1(p) (*((signed char*)(p)))
    static uint16_t U2(unsigned char* p)
    {
        uint16_t u = 0;
        memcpy(&u, p, 2);
        return u;
    }
    static unsigned int U4(unsigned char* p)
    {
        unsigned int u = 0;
        memcpy(&u, p, 4);
        return u;
    }
    static int I4(unsigned char* p)
    {
        int u = 0;
        memcpy(&u, p, 4);
        return u;
    }
    static float R4(unsigned char* p)
    {
        float r = 0.0F;
        memcpy(&r, p, 4);
        return r;
    }
    static double R8(unsigned char* p)
    {
        double r = 0.0;
        memcpy(&r, p, 8);
        return r;
    }

    static double I8(unsigned char* p)
    {
        return I4(p + 4) * 4294967296.0 + U4(p);
    }

  private:
    void ensureCanExtract(size_t numOfBytes);

    std::vector<unsigned char> _data;
    size_t _curExtractLoc = 0;
};

} // namespace NAV::ublox
