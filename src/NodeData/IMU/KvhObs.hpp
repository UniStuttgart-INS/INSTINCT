/// @file KvhObs.hpp
/// @brief Data storage class for one KVH Imu observation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-30

#pragma once

#include "ImuObs.hpp"
#include "uart/protocol/packet.hpp"
#include <bitset>

namespace NAV
{
/// Kvh Observation storage Class
class KvhObs final : public ImuObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    /// @param[in] packet The packet to copy into the raw data
    KvhObs(const ImuPos& imuPos, uart::protocol::Packet& packet)
        : ImuObs(imuPos), raw(packet) {}

    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit KvhObs(const ImuPos& imuPos)
        : ImuObs(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("KvhObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// Complete message raw binary data including header and checksum
    uart::protocol::Packet raw;

    /// @brief Status Byte
    ///
    /// Bit | Function               | Notes
    ///  0  | Gyro X status          | 1 = Valid data, 0 = Invalid data
    ///  1  | Gyro Y status          | 1 = Valid data, 0 = Invalid data
    ///  2  | Gyro Z status          | 1 = Valid data, 0 = Invalid data
    ///  3  | Reserved               | Always 0
    ///  4  | Accelerometer X status | 1 = Valid data, 0 = Invalid data
    ///  5  | Accelerometer Y status | 1 = Valid data, 0 = Invalid data
    ///  6  | Accelerometer Z status | 1 = Valid data, 0 = Invalid data
    ///  7  | Reserved               | Always 0
    std::bitset<8> status{};

    /// Increments for each message and resets to 0 after 127
    uint8_t sequenceNumber = UINT8_MAX;
};

} // namespace NAV
