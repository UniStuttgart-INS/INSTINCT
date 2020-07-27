/**
 * @file UbloxUartSensor.hpp
 * @brief Class to read out Ublox Sensors
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-07-22
 */

#include "uart/sensors/sensors.hpp"

namespace NAV::sensors
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
    uart::sensors::UartSensor& operator->() { return sensor; };

  private:
    uart::sensors::UartSensor sensor{ endianness,
                                      packageFinderFunction,
                                      packetTypeFunction,
                                      checksumFunction,
                                      isErrorFunction,
                                      isResponseFunction,
                                      packageHeaderLength };

    static constexpr uart::Endianness endianness = uart::Endianness::ENDIAN_LITTLE;
    static constexpr size_t packageHeaderLength = 2;

    static void packageFinderFunction(const std::vector<uint8_t>& data,
                                      const uart::xplat::TimeStamp& timestamp,
                                      uart::sensors::UartSensor::ValidPacketFoundHandler dispatchPacket, void* dispatchPacketUserData,
                                      uart::sensors::UartSensor* uartSensor);

    static uart::protocol::Packet::Type packetTypeFunction(const uart::protocol::Packet& packet);

    static bool checksumFunction(const uart::protocol::Packet& packet);

    static bool isErrorFunction(const uart::protocol::Packet& packet);

    static bool isResponseFunction(const uart::protocol::Packet& packet);
};

} // namespace NAV::sensors
