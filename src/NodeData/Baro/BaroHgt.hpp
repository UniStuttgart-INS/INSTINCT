// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file BaroHgt.hpp
/// @brief Barometric Height Storage Class
/// @author T. Hobiger (hobiger@ins.uni-stuttgart.de)
/// @date 2025-02-10

#pragma once

#include "NodeData/NodeData.hpp"
#include "util/Assert.h"
#include <Eigen/src/Core/MatrixBase.h>

namespace NAV
{
/// Barometric height storage class
class BaroHgt : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "BaroHgt";
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

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        return {
            "BaroHgt [m]",
            "BaroHgt StDev [m]"
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 2; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0: // BaroHgt [m]
            return baro_height;
        case 1: // BaroHgt StDev [m]
            return baro_heightStdev;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Set the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @param value Value to set
    /// @return True if the value was updated
    [[nodiscard]] bool setValueAt(size_t idx, double value) override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0: // BaroHgt [m]
            baro_height = value;
            break;
        case 1: // BaroHgt StDev [m]
        default:
            return false;
        }

        return true;
    }

    /// Barometric height [m]
    double baro_height{ std::nan("") };

    /// Standard deviation of barometric height [m]
    std::optional<double> baro_heightStdev;
};

} // namespace NAV
