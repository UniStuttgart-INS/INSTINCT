// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NodeData.hpp
/// @brief Abstract NodeData Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-16

#pragma once

#include <string>
#include <vector>
#include <optional>

#include "Navigation/Time/InsTime.hpp"

namespace NAV
{
/// @brief Parent class for all data transmitted over Flow pins
class NodeData
{
  public:
    /// @brief Default constructor
    NodeData() = default;
    /// @brief Destructor
    virtual ~NodeData() = default;
    /// @brief Copy constructor
    NodeData(const NodeData&) = default;
    /// @brief Move constructor
    NodeData(NodeData&&) = default;
    /// @brief Copy assignment operator
    NodeData& operator=(const NodeData&) = default;
    /// @brief Move assignment operator
    NodeData& operator=(NodeData&&) = default;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type() { return "NodeData"; }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes() { return {}; }

    /// Time at which the message was received
    InsTime insTime;
};

} // namespace NAV
