/// @file EmlidObs.hpp
/// @brief Emlid Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "GnssObs.hpp"

#include <variant>

#include "uart/protocol/packet.hpp"

#include "util/UartSensors/Emlid/EmlidTypes.hpp"

namespace NAV
{
/// Emlid Observation Class
class EmlidObs : public GnssObs
{
  public:
    /// @brief Constructor
    /// @param[in] packet The packet to copy into the raw data
    explicit EmlidObs(uart::protocol::Packet& packet)
        : raw(packet) {}

    /// @brief Default constructor
    EmlidObs() = default;
    /// @brief Destructor
    ~EmlidObs() override = default;
    /// @brief Copy constructor
    EmlidObs(const EmlidObs&) = delete;
    /// @brief Move constructor
    EmlidObs(EmlidObs&&) = delete;
    /// @brief Copy assignment operator
    EmlidObs& operator=(const EmlidObs&) = delete;
    /// @brief Move assignment operator
    EmlidObs& operator=(EmlidObs&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static constexpr std::string_view type()
    {
        return std::string_view("EmlidObs");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string_view> parentTypes()
    {
        std::vector<std::string_view> parents{ "GnssObs" };
        return parents;
    }

    /// Erb Message ID
    uint8_t msgId = 0;
    /// Payload length in bytes
    uint16_t payloadLength = 0;

    /// Complete message raw binary data including header and checksum (ERB)
    uart::protocol::Packet raw;

    /// Decoded data
    std::variant<
        sensors::emlid::ErbVer,  // VER: Version of protocol
        sensors::emlid::ErbPos,  // POS: Geodetic position solution
        sensors::emlid::ErbStat, // STAT: Receiver navigation status
        sensors::emlid::ErbDops, // DOPS: Dilution of precision
        sensors::emlid::ErbVel,  // VEL: Velocity solution in NED
        sensors::emlid::ErbSvi,  // SVI: Space vehicle information
        sensors::emlid::ErbRtk   // RTK: RTK information
        >
        data{};
};

} // namespace NAV
