// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MmaeSolution.hpp
/// @brief MMAE solution information
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2025-07-23

#pragma once

#include "InsGnssLCKFSolution.hpp"

namespace NAV
{
/// MMAE solution information
class MmaeSolution : public InsGnssLCKFSolution
{
  public:
    struct MmaeSolutionData
    {
        size_t kfId{};
        /// MMAE weight (probability)
        double weight = 0.0;
    };

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "MmaeSolution";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = InsGnssLCKFSolution::parentTypes();
        parent.push_back(InsGnssLCKFSolution::type());
        return parent;
    }

    std::vector<MmaeSolutionData> data;

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(data.size());

        for (const auto& solData : data)
        {
            descriptors.push_back(fmt::format("KF {} MMAE weight [-]", solData.kfId + 1));
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& solData : data)
        {
            if (descriptor == fmt::format("KF {} MMAE weight [-]", solData.kfId + 1))
            {
                return solData.weight;
            }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(data.size());
        for (const auto& solData : data)
        {
            dynData.emplace_back(fmt::format("KF {} MMAE weight [-]", solData.kfId + 1), solData.weight);
        }
        return dynData;
    }
};
} // namespace NAV