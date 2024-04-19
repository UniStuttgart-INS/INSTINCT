// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuObsWDelta.hpp
/// @brief Data storage class for one VectorNavImu observation
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "ImuObs.hpp"

namespace NAV
{
/// VectorNav Observation storage Class
class ImuObsWDelta final : public ImuObs
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObsWDelta(const ImuPos& imuPos)
        : ImuObs(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ImuObsWDelta";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { ImuObs::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = ImuObs::GetStaticDataDescriptors();
        desc.emplace_back("dTime [s]");
        desc.emplace_back("dTheta X [deg]");
        desc.emplace_back("dTheta Y [deg]");
        desc.emplace_back("dTheta Z [deg]");
        desc.emplace_back("dVelocity X [m/s]");
        desc.emplace_back("dVelocity Y [m/s]");
        desc.emplace_back("dVelocity Z [m/s]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 27; }

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
        case 0:  // Time since startup [ns]
        case 1:  // Mag uncomp X [Gauss]
        case 2:  // Mag uncomp Y [Gauss]
        case 3:  // Mag uncomp Z [Gauss]
        case 4:  // Accel uncomp X [m/s^2]
        case 5:  // Accel uncomp Y [m/s^2]
        case 6:  // Accel uncomp Z [m/s^2]
        case 7:  // Gyro uncomp X [rad/s]
        case 8:  // Gyro uncomp Y [rad/s]
        case 9:  // Gyro uncomp Z [rad/s]
        case 10: // Mag Comp X [Gauss]
        case 11: // Mag Comp Y [Gauss]
        case 12: // Mag Comp Z [Gauss]
        case 13: // Accel Comp X [m/s^2]
        case 14: // Accel Comp Y [m/s^2]
        case 15: // Accel Comp Z [m/s^2]
        case 16: // Gyro Comp X [rad/s]
        case 17: // Gyro Comp Y [rad/s]
        case 18: // Gyro Comp Z [rad/s]
        case 19: // Temperature [°C]
            return ImuObs::getValueAt(idx);
        case 20: // dTime [s]
            return dtime;
        case 21: // dTheta X [deg]
            if (dtheta.has_value()) { return dtheta->x(); }
            break;
        case 22: // dTheta Y [deg]
            if (dtheta.has_value()) { return dtheta->y(); }
            break;
        case 23: // dTheta Z [deg]
            if (dtheta.has_value()) { return dtheta->z(); }
            break;
        case 24: // dVelocity X [m/s]
            if (dvel.has_value()) { return dvel->x(); }
            break;
        case 25: // dVelocity Y [m/s]
            if (dvel.has_value()) { return dvel->y(); }
            break;
        case 26: // dVelocity Z [m/s]
            if (dvel.has_value()) { return dvel->z(); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// The time interval that the delta angle and velocities are integrated over in [seconds].
    double dtime{ std::nan("") };
    /// The delta rotation angles in [degree] incurred due to rotation, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Vector3d> dtheta;
    /// The delta velocity in [m/s] incurred due to motion, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    std::optional<Eigen::Vector3d> dvel;
};

} // namespace NAV
