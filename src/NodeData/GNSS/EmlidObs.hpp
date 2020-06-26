/**
 * @file EmlidObs.hpp
 * @brief Emlid Observation Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-06-23
 */

#pragma once

#include "GnssObs.hpp"

#include <variant>

#include "util/Emlid/EmlidTypes.hpp"
#include "util/Emlid/EmlidPacket.hpp"

namespace NAV
{
/// Emlid Observation Class
class EmlidObs : public GnssObs
{
  public:
    EmlidObs() = default;                          ///< Constructor
    ~EmlidObs() override = default;                ///< Destructor
    EmlidObs(const EmlidObs&) = delete;            ///< Copy constructor
    EmlidObs(EmlidObs&&) = delete;                 ///< Move constructor
    EmlidObs& operator=(const EmlidObs&) = delete; ///< Copy assignment operator
    EmlidObs& operator=(EmlidObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("EmlidObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "GnssObs" };
        return parents;
    }

    /// Erb Message ID
    uint8_t msgId = 0;
    /// Payload length in bytes
    uint16_t payloadLength = 0;

    /// Complete message raw binary data including header and checksum (ERB)
    Emlid::EmlidPacket raw;

    /// Decoded data
    std::variant<
        Emlid::ErbVer,  // VER: Version of protocol
        Emlid::ErbPos,  // POS: Geodetic position solution
        Emlid::ErbStat, // STAT: Receiver navigation status
        Emlid::ErbDops, // DOPS: Dilution of precision
        Emlid::ErbVel,  // VEL: Velocity solution in NED
        Emlid::ErbSvi,  // SVI: Space vehicle information
        Emlid::ErbRtk   // RTK: RTK information
        >
        data{};
};

} // namespace NAV
