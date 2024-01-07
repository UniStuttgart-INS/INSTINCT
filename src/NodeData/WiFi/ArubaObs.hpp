// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ArubaObs.hpp
/// @brief Aruba Observation Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date TODO

#pragma once

#include "NodeData/NodeData.hpp"

namespace NAV
{
/// Aruba Observation Class
class ArubaObs : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ArubaObs";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    struct FtmObs
    {
        std::string macAddress;
        InsTime time;
        double measuredDistance;
    };

    std::vector<FtmObs> data;
};

} // namespace NAV
