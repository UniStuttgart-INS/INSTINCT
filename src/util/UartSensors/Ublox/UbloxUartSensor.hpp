/**
 * @file UbloxUartSensor.hpp
 * @brief Class to read out Ublox Sensors
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-07-22
 */

#pragma once

#include <memory>

#include "uart/sensors/sensors.hpp"

namespace NAV::sensors::ublox
{
class UbloxUartSensor
{
  public:
    /// @brief Default constructor
    UbloxUartSensor() = default;
    /// @brief Destructor
    ~UbloxUartSensor() = default;
    /// @brief Copy constructor
    UbloxUartSensor(const UbloxUartSensor&) = delete;
    /// @brief Move constructor
    UbloxUartSensor(UbloxUartSensor&&) = delete;
    /// @brief Copy assignment operator
    UbloxUartSensor& operator=(const UbloxUartSensor&) = delete;
    /// @brief Move assignment operator
    UbloxUartSensor& operator=(UbloxUartSensor&&) = delete;
    /// @brief Arrow operator overload
    uart::sensors::UartSensor* operator->() { return &sensor; };

    std::unique_ptr<uart::protocol::Packet> findPacket(uint8_t dataByte, uart::sensors::UartSensor* uartSensor);

  private:
    uart::sensors::UartSensor sensor{ endianness,
                                      packetFinderFunction,
                                      this,
                                      packetTypeFunction,
                                      checksumFunction,
                                      isErrorFunction,
                                      isResponseFunction,
                                      packetHeaderLength };

    static constexpr uart::Endianness endianness = uart::Endianness::ENDIAN_LITTLE;
    static constexpr size_t packetHeaderLength = 2;

    static void packetFinderFunction(const std::vector<uint8_t>& data,
                                     const uart::xplat::TimeStamp& timestamp,
                                     uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData,
                                     uart::sensors::UartSensor* uartSensor,
                                     void* userData);

    static uart::protocol::Packet::Type packetTypeFunction(const uart::protocol::Packet& packet);

    static bool checksumFunction(const uart::protocol::Packet& packet);

    static bool isErrorFunction(const uart::protocol::Packet& packet);

    static bool isResponseFunction(const uart::protocol::Packet& packet);

    std::vector<uint8_t> _buffer;
    size_t _bufferAppendLocation = 0;
};

} // namespace NAV::sensors::ublox
