// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InsGnssTCKFSolution.hpp
/// @brief Tightly-coupled Kalman Filter INS/GNSS errors
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-02-24

#pragma once

#include "InsGnssLCKFSolution.hpp"

namespace NAV
{
/// Tightly-coupled Kalman Filter INS/GNSS errors
class InsGnssTCKFSolution : public InsGnssLCKFSolution
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "InsGnssTCKFSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = InsGnssLCKFSolution::parentTypes();
        parent.push_back(InsGnssLCKFSolution::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = InsGnssLCKFSolution::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Receiver clock offset [m]");
        desc.emplace_back("Receiver clock drift [m/s]");
        desc.emplace_back("Receiver clock offset [s]");
        desc.emplace_back("Receiver clock drift [s/s]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 28; }

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
        case 0:  // Roll error [deg]
        case 1:  // Pitch error [deg]
        case 2:  // Yaw error [deg]
        case 3:  // North velocity error [m/s]
        case 4:  // East velocity error [m/s]
        case 5:  // Down velocity error [m/s]
        case 6:  // Latitude error [deg]
        case 7:  // Longitude error [deg]
        case 8:  // Altitude error [m]
        case 9:  // Alpha_eb [deg]
        case 10: // Beta_eb [deg]
        case 11: // Gamma_eb [deg]
        case 12: // ECEF X velocity error [m/s]
        case 13: // ECEF Y velocity error [m/s]
        case 14: // ECEF Z velocity error [m/s]
        case 15: // ECEF X error [m]
        case 16: // ECEF Y error [m]
        case 17: // ECEF Z error [m]
        case 18: // Accelerometer bias b_X [m/s^2]
        case 19: // Accelerometer bias b_Y [m/s^2]
        case 20: // Accelerometer bias b_Z [m/s^2]
        case 21: // Gyroscope bias b_X [rad/s]
        case 22: // Gyroscope bias b_Y [rad/s]
        case 23: // Gyroscope bias b_Z [rad/s]
            return InsGnssLCKFSolution::getValueAt(idx);
        case 24: // Receiver clock offset [m]
            return recvClkOffset;
        case 25: // Receiver clock drift [m/s]
            return recvClkDrift;
        case 26: // Receiver clock offset [s]
            return recvClkOffset / InsConst<>::C;
        case 27: // Receiver clock drift [s/s]
            return recvClkDrift / InsConst<>::C;
        default:
            return std::nullopt;
        }
    }

    /// δϱ The receiver clock offset in [m]
    double recvClkOffset{};

    /// δϱ_dot The receiver clock drift in [m/s]
    double recvClkDrift{};
};

} // namespace NAV