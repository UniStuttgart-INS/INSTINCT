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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 74; }

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
        case 0:  // Latitude [deg]
        case 1:  // Longitude [deg]
        case 2:  // Altitude [m]
        case 3:  // North/South [m]
        case 4:  // East/West [m]
        case 5:  // X-ECEF [m]
        case 6:  // Y-ECEF [m]
        case 7:  // Z-ECEF [m]
        case 8:  // X-ECEF StDev [m]
        case 9:  // Y-ECEF StDev [m]
        case 10: // Z-ECEF StDev [m]
        case 11: // XY-ECEF StDev [m]
        case 12: // XZ-ECEF StDev [m]
        case 13: // YZ-ECEF StDev [m]
        case 14: // North StDev [m]
        case 15: // East StDev [m]
        case 16: // Down StDev [m]
        case 17: // NE StDev [m]
        case 18: // ND StDev [m]
        case 19: // ED StDev [m]
        case 20: // Velocity norm [m/s]
        case 21: // X velocity ECEF [m/s]
        case 22: // Y velocity ECEF [m/s]
        case 23: // Z velocity ECEF [m/s]
        case 24: // North velocity [m/s]
        case 25: // East velocity [m/s]
        case 26: // Down velocity [m/s]
        case 27: // X velocity ECEF StDev [m/s]
        case 28: // Y velocity ECEF StDev [m/s]
        case 29: // Z velocity ECEF StDev [m/s]
        case 30: // XY velocity StDev [m]
        case 31: // XZ velocity StDev [m]
        case 32: // YZ velocity StDev [m]
        case 33: // North velocity StDev [m/s]
        case 34: // East velocity StDev [m/s]
        case 35: // Down velocity StDev [m/s]
        case 36: // NE velocity StDev [m]
        case 37: // ND velocity StDev [m]
        case 38: // ED velocity StDev [m]
        case 39: // Roll [deg]
        case 40: // Pitch [deg]
        case 41: // Yaw [deg]
        case 42: // Quaternion::w
        case 43: // Quaternion::x
        case 44: // Quaternion::y
        case 45: // Quaternion::z
        case 46: // KF State Roll error [deg]
        case 47: // KF State Pitch error [deg]
        case 48: // KF State Yaw error [deg]
        case 49: // KF State Position North error [deg]
        case 50: // KF State Position East error [deg]
        case 51: // KF State Position Down error [m]
        case 52: // KF State Velocity North error [m/s]
        case 53: // KF State Velocity East error [m/s]
        case 54: // KF State Velocity Down error [m/s]
        case 55: // KF State Alpha_eb [deg]
        case 56: // KF State Beta_eb [deg]
        case 57: // KF State Gamma_eb [deg]
        case 58: // KF State Position ECEF X error [m]
        case 59: // KF State Position ECEF Y error [m]
        case 60: // KF State Position ECEF Z error [m]
        case 61: // KF State Velocity ECEF X error [m/s]
        case 62: // KF State Velocity ECEF Y error [m/s]
        case 63: // KF State Velocity ECEF Z error [m/s]
        case 64: // Accelerometer bias b_X [m/s^2]
        case 65: // Accelerometer bias b_Y [m/s^2]
        case 66: // Accelerometer bias b_Z [m/s^2]
        case 67: // Gyroscope bias b_X [rad/s]
        case 68: // Gyroscope bias b_Y [rad/s]
        case 69: // Gyroscope bias b_Z [rad/s]
            return InsGnssLCKFSolution::getValueAt(idx);
        case 70: // Receiver clock offset [m]
            return recvClkOffset;
        case 71: // Receiver clock drift [m/s]
            return recvClkDrift;
        case 72: // Receiver clock offset [s]
            return recvClkOffset / InsConst<>::C;
        case 73: // Receiver clock drift [s/s]
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