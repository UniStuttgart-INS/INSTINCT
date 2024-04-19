// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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
        return "KvhObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = ImuObs::GetStaticDataDescriptors();
        desc.emplace_back("Status [bits]");
        desc.emplace_back("Sequence Number [-]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 22; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0:  // Time since startup [ns]
        case 1:  // Mag uncomp X [Gauss]
        case 2:  // Mag uncomp Y [Gauss]
        case 3:  // Mag uncomp Z [Gauss]
        case 4:  // Accel uncomp X [m/s^2]
        case 5:  // Accel uncomp Y [m/s^2]
        case 6:  // Accel uncomp Z [m/s^2]
        case 7:  // Gyro uncomp X [rad/s]
        case 8:  // Gyro uncomp Y [rad/s]
        case 9:  // Gyro uncomp Z [rad/s]
        case 10: // Mag Comp X [Gauss]
        case 11: // Mag Comp Y [Gauss]
        case 12: // Mag Comp Z [Gauss]
        case 13: // Accel Comp X [m/s^2]
        case 14: // Accel Comp Y [m/s^2]
        case 15: // Accel Comp Z [m/s^2]
        case 16: // Gyro Comp X [rad/s]
        case 17: // Gyro Comp Y [rad/s]
        case 18: // Gyro Comp Z [rad/s]
        case 19: // Temperature [°C]
            return ImuObs::getValueAt(idx);
        case 20: // Status [bits]
            return static_cast<double>(status.to_ulong());
        case 21: // Sequence Number [-]
            if (sequenceNumber < 128) { return sequenceNumber; }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
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
