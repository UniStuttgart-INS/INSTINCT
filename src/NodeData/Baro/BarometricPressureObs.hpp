// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file BarometricPressureObs.hpp
/// @brief Barometric Pressure Storage Class
/// @author T. Hobiger (hobiger@ins.uni-stuttgart.de)
/// @date 2025-02-10

#pragma once

#include "NodeData/NodeData.hpp"
#include "util/Assert.h"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class BarometricPressureObs : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "BarometricPressureObs";
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
            "BarometricPressureObs [hPa]"
        };
    }

    /// @brief Get the number of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 1; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the number of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (idx == 0) // BarometricPressureObs [hPa]
        {
            return baro_pressure;
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
        if (idx == 0)
        {
            baro_pressure = value;
        }
        else
        {
            return false;
        }

        return true;
    }

    /// Barometric pressure [hPa]
    double baro_pressure{ std::nan("") };
};

} // namespace NAV
