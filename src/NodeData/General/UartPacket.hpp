// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UartPacket.hpp
/// @brief UART Packet storage class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-13

#pragma once

#include "NodeData/NodeData.hpp"

#include "uart/protocol/packet.hpp"

namespace NAV
{
/// UART Packet storage class
class UartPacket : public NodeData
{
  public:
    /// @brief Constructor
    /// @param[in] packet The packet to copy into the raw data
    explicit UartPacket(uart::protocol::Packet& packet)
        : raw(packet) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "UartPacket";
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

    /// Complete message raw binary data
    uart::protocol::Packet raw;
};

} // namespace NAV