/**
 * @file KvhObs.hpp
 * @brief Data storage class for one KVH Imu observation
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-06-30
 */

#pragma once

#include "ImuObs.hpp"
#include "util/KVH/KvhPacket.hpp"
#include <bitset>

namespace NAV
{
/// Kvh Observation storage Class
class KvhObs final : public ImuObs
{
  public:
    KvhObs() = default;                        ///< Constructor
    ~KvhObs() final = default;                 ///< Destructor
    KvhObs(const KvhObs&) = delete;            ///< Copy constructor
    KvhObs(KvhObs&&) = delete;                 ///< Move constructor
    KvhObs& operator=(const KvhObs&) = delete; ///< Copy assignment operator
    KvhObs& operator=(KvhObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("KvhObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "ImuObs" };
        return parents;
    }

    /// Complete message raw binary data including header and checksum
    KvhPacket raw;

    /** Time since startup of the IMU [µs] */
    std::optional<uint32_t> timeSinceStartup;

    /** The IMU temperature measured in units of [Celsius]. */
    std::optional<double> temperature = 0.0;

    /** Status Byte
     * 
     * Bit | Function               | Notes
     *  0  | Gyro X status          | 1 = Valid data, 0 = Invalid data
     *  1  | Gyro Y status          | 1 = Valid data, 0 = Invalid data
     *  2  | Gyro Z status          | 1 = Valid data, 0 = Invalid data
     *  3  | Reserved               | Always 0
     *  4  | Accelerometer X status | 1 = Valid data, 0 = Invalid data
     *  5  | Accelerometer Y status | 1 = Valid data, 0 = Invalid data
     *  6  | Accelerometer Z status | 1 = Valid data, 0 = Invalid data
     *  7  | Reserved               | Always 0
     */
    std::bitset<8> status{};

    /** Increments for each message and resets to 0 after 127 */
    uint8_t sequenceNumber = UINT8_MAX;
};

} // namespace NAV
