// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file EmlidObs.hpp
/// @brief Emlid Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "NodeData/NodeData.hpp"

#include <variant>

#include "util/Vendor/Emlid/EmlidTypes.hpp"

namespace NAV
{
/// Emlid Observation Class
class EmlidObs : public NodeData
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
        return { NodeData::type() };
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
        data;
};

} // namespace NAV
