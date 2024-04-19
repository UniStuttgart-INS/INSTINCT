// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DynamicData.hpp
/// @brief Dynamic Data container
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-29

#pragma once

#include <vector>
#include <string>

#include "NodeData/NodeData.hpp"

namespace NAV
{
/// Dynamic Data container
class DynamicData : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type() { return "DynamicData"; }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// Data struct
    struct Data
    {
        std::string description;         ///< Description
        double value;                    ///< Value
        std::vector<std::string> events; ///< List of events
    };

    /// @brief Data storage
    std::vector<Data> data;

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(data.size());

        for (const auto& d : data)
        {
            descriptors.push_back(d.description);
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& d : data)
        {
            if (descriptor == d.description) { return d.value; }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(data.size());

        for (const auto& d : data)
        {
            dynData.emplace_back(d.description, d.value);
        }
        return dynData;
    }
};

} // namespace NAV
