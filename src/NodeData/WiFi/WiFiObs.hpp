// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WiFiObs.hpp
/// @brief Espressif Observation Class
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @date 2024-01-08

#pragma once

#include "NodeData/NodeData.hpp"
#include "util/Vendor/VectorNav/BinaryOutputs/TimeOutputs.hpp"

namespace NAV
{
/// Espressif Observation Class
class WiFiObs : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "WiFiObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// Payload length in bytes
    uint16_t payloadLength = 0;

    /// MAC address of the device
    std::string macAddress;
    /// Distance to the device
    double distance;
    /// Standard deviation of the distance
    double distanceStd;
    /// Time of observation
    NAV::vendor::vectornav::TimeOutputs timeOutputs;
};

} // namespace NAV
