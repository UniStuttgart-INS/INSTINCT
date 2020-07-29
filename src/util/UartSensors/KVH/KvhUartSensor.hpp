/// @file KvhUartSensor.hpp
/// @brief Class to read out KVH Sensors
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-28

#pragma once

#include <memory>

#include "uart/sensors/sensors.hpp"

namespace NAV::sensors::kvh
{
class KvhUartSensor
{
  public:
    /// @brief Default constructor
    KvhUartSensor();
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
    uart::sensors::UartSensor* operator->() { return &sensor; };

    /// @brief Collects data bytes and searches for packages inside of them
    /// @param[in] dataByte The next data byte
    /// @return nullptr if no packet found yet, otherwise a pointer to the packet
    std::unique_ptr<uart::protocol::Packet> findPacket(uint8_t dataByte);

    static constexpr uint32_t HEADER_FMT_A = 0xFE81FF55;
    static constexpr uint32_t HEADER_FMT_B = 0xFE81FF56;
    static constexpr uint32_t HEADER_FMT_C = 0xFE81FF57;
    static constexpr uint32_t HEADER_FMT_XBIT = 0xFE8100AA;
    static constexpr uint32_t HEADER_FMT_XBIT2 = 0xFE8100AB;

  private:
    uart::sensors::UartSensor sensor{ endianness,
                                      packetFinderFunction,
                                      this,
                                      packetTypeFunction,
                                      checksumFunction,
                                      isErrorFunction,
                                      isResponseFunction,
                                      packetHeaderLength };

    static constexpr uart::Endianness endianness = uart::Endianness::ENDIAN_BIG;
    static constexpr size_t packetHeaderLength = 0;

    static void packetFinderFunction(const std::vector<uint8_t>& data,
                                     const uart::xplat::TimeStamp& timestamp,
                                     uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData,
                                     void* userData);

    static uart::protocol::Packet::Type packetTypeFunction(const uart::protocol::Packet& packet);

    static bool checksumFunction(const uart::protocol::Packet& packet);

    static bool isErrorFunction(const uart::protocol::Packet& packet);

    static bool isResponseFunction(const uart::protocol::Packet& packet);

    static constexpr uint8_t AsciiEndChar1 = '\r';
    static constexpr uint8_t AsciiEndChar2 = '\n';
    static constexpr uint8_t AsciiEscapeChar = '\0';
    static constexpr size_t MaximumSizeForAsciiPacket = 256;

    bool currentlyBuildingAsciiPacket{ false };
    bool currentlyBuildingBinaryPacket{ false };

    bool asciiEndChar1Found{ false };

    enum TagState
    {
        SM_H1,
        SM_H2,
        SM_H3,
        SM_X3,
        SM_IDLE
    };

    TagState eState = SM_IDLE;

    enum HeaderType
    {
        FMT_A,
        FMT_B,
        FMT_C,
        FMT_XBIT,
        FMT_XBIT2,
        FMT_UNKNOWN
    };

    HeaderType packetType = HeaderType::FMT_UNKNOWN;

    HeaderType bFindImuHeader(uint8_t ui8Data);

    std::vector<uint8_t> _buffer;

    /// Used for correlating raw data with where the packet was found for the end user.
    size_t runningDataIndex{ 0 };
    size_t numOfBytesRemainingForCompletePacket{ 0 };

    void resetTracking();
};

} // namespace NAV::sensors::kvh
