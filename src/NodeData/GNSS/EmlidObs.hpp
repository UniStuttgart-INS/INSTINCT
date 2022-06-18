/// @file EmlidObs.hpp
/// @brief Emlid Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "NodeData/InsObs.hpp"

#include <variant>

#include "util/Vendor/Emlid/EmlidTypes.hpp"

namespace NAV
{
/// Emlid Observation Class
class EmlidObs : public InsObs
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "EmlidObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { InsObs::type() };
    }

    /// Erb Message ID
    uint8_t msgId = 0;
    /// Payload length in bytes
    uint16_t payloadLength = 0;

    /// Decoded data
    std::variant<
        vendor::emlid::ErbVer,  // VER: Version of protocol
        vendor::emlid::ErbPos,  // POS: Geodetic position solution
        vendor::emlid::ErbStat, // STAT: Receiver navigation status
        vendor::emlid::ErbDops, // DOPS: Dilution of precision
        vendor::emlid::ErbVel,  // VEL: Velocity solution in NED
        vendor::emlid::ErbSvi,  // SVI: Space vehicle information
        vendor::emlid::ErbRtk   // RTK: RTK information
        >
        data{};
};

} // namespace NAV
