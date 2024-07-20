// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WiFiPositioningSolution.hpp
/// @brief WiFi Positioning Algorithm output
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-03-12

#pragma once

#include "NodeData/State/PosVel.hpp"
#include <Eigen/Dense>

namespace NAV
{
class WiFiPositioningSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "WiFiPositioningSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVel::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Bias [m]");
        desc.emplace_back("Bias StDev [m]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 41; }

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
            return PosVel::getValueAt(idx);
        case 39: // Bias [m]
            return bias;
        case 40: // Bias StDev [m]
            return biasStdev;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    // --------------------------------------------------------- Public Members ------------------------------------------------------------
    /// Bias [m]
    double bias = std::nan("");
    /// Standard deviation of Bias [m]
    double biasStdev = std::nan("");

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    Eigen::Vector3d _e_positionStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Position in local navigation frame coordinates [m]
    Eigen::Vector3d _n_positionStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Standard deviation of Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Covariance matrix in ECEF coordinates (Position, Velocity)
    Eigen::MatrixXd _e_covarianceMatrix;
    /// Covariance matrix in local navigation coordinates (Position, Velocity)
    Eigen::MatrixXd _n_covarianceMatrix;
};

} // namespace NAV
