// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file packet.h
/// @brief Extract from the vnproglib
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-09-20

#pragma once

#ifndef HAS_VECTORNAV_LIBRARY

    #include <array>
    #include <cstdint>
    #include "vector.h"
    #include "matrix.h"
    #include "types.h"

// NOLINTBEGIN

namespace vn
{
namespace protocol
{
namespace uart
{

/// \brief Structure representing a UART packet received from the VectorNav
/// sensor.
struct Packet
{
  public:
    /// \brief The different types of UART packets.
    enum Type
    {
        TYPE_UNKNOWN, ///< Type is unknown.
        TYPE_BINARY,  ///< Binary packet.
        TYPE_ASCII    ///< ASCII packet.
    };

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

    /// \brief Indicates if the packet is an ASCII asynchronous message.
    ///
    /// \return <c>true</c> if the packet is an ASCII asynchronous message;
    ///     otherwise <c>false</c>.
    bool isAsciiAsync();

    /// \brief Determines the type of ASCII asynchronous message this packet
    /// is.
    ///
    /// \return The asynchronous data type of the packet.
    AsciiAsync determineAsciiAsyncType();

    /// \brief Determines if the packet is a compatible match for an expected
    /// binary output message type.
    ///
    /// \param[in] commonGroup The Common Group configuration.
    /// \param[in] timeGroup The Time Group configuration.
    /// \param[in] imuGroup The IMU Group configuration.
    /// \param[in] gpsGroup The GPS Group configuration.
    /// \param[in] attitudeGroup The Attitude Group configuration.
    /// \param[in] insGroup The INS Group configuration.
    /// \return <c>true</c> if the packet matches the expected group
    ///     configuration; otherwise <c>false</c>.
    bool isCompatible(CommonGroup commonGroup, TimeGroup timeGroup, ImuGroup imuGroup, GpsGroup gpsGroup, AttitudeGroup attitudeGroup, InsGroup insGroup, GpsGroup gps2Group);

    /// \brief Parses an error packet to get the error type.
    ///
    /// \return The sensor error.
    SensorError parseError();

    /// \brief If the packet is a binary message, this will return the groups field.
    /// \return The present groups field.
    uint8_t groups();

    /// \brief This will return the requested group field of a binary packet at the
    /// specified index.
    ///
    /// \param[in] index The 0-based index of the requested group field.
    /// \return The group field.
    uint16_t groupField(size_t index);

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

    /// \brief Extracts a int16_t data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    int8_t extractInt16();

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

    /// \brief Extracts a float data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    float extractFloat();

    /// \brief Extracts a double data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    double extractDouble();

    /// \brief Extracts a vec3f data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    vn::math::vec3f extractVec3f();

    /// \brief Extracts a vec3d data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    vn::math::vec3d extractVec3d();

    /// \brief Extracts a vec4f data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    vn::math::vec4f extractVec4f();

    /// \brief Extract a mat3f data type from a binary packet and advances
    /// the next extraction point appropriately.
    ///
    /// \return The extracted value.
    vn::math::mat3f extractMat3f();

    /// \}

    /// \brief Get the Packet Length
    size_t getPacketLength();

    /// \brief Get the Current Extract Location
    size_t getCurExtractLoc();

  private:
    void ensureCanExtract(size_t numOfBytes);
};

} // namespace uart
} // namespace protocol
} // namespace vn

// NOLINTEND

#endif