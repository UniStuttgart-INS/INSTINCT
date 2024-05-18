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
class ImuObsWDelta : public ImuObs
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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 18; }

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
        case 1:  // Accel X [m/s^2]
        case 2:  // Accel Y [m/s^2]
        case 3:  // Accel Z [m/s^2]
        case 4:  // Gyro X [rad/s]
        case 5:  // Gyro Y [rad/s]
        case 6:  // Gyro Z [rad/s]
        case 7:  // Mag X [Gauss]
        case 8:  // Mag Y [Gauss]
        case 9:  // Mag Z [Gauss]
        case 10: // Temperature [°C]
            return ImuObs::getValueAt(idx);
        case 11: // dTime [s]
            return dtime;
        case 12: // dTheta X [deg]
            return dtheta.x();
        case 13: // dTheta Y [deg]
            return dtheta.y();
        case 14: // dTheta Z [deg]
            return dtheta.z();
        case 15: // dVelocity X [m/s]
            return dvel.x();
        case 16: // dVelocity Y [m/s]
            return dvel.y();
        case 17: // dVelocity Z [m/s]
            return dvel.z();
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// The time interval that the delta angle and velocities are integrated over in [seconds].
    double dtime = 0.0;
    /// The delta rotation angles in [degree] incurred due to rotation, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    Eigen::Vector3d dtheta;
    /// The delta velocity in [m/s] incurred due to motion, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    Eigen::Vector3d dvel;
};

} // namespace NAV
