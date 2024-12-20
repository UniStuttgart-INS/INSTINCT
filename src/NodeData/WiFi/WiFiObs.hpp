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

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// MAC address of the device
    std::string macAddress;
    /// Distance to the device
    double distance = 0;
    /// Standard deviation of the distance
    double distanceStd = 0;
    /// Struct for time sync
    struct TimeSyncInput
    {
        /// @brief Time since last SyncIn trigger.
        ///
        /// The time since the last SyncIn event trigger expressed in nano seconds.
        uint64_t timeSyncIn{};
        /// @brief SyncIn trigger count.
        ///
        /// The number of SyncIn trigger events that have occurred.
        uint32_t syncInCnt{};
    };
    /// Time of observation
    TimeSyncInput timeOutputs;
};

} // namespace NAV
