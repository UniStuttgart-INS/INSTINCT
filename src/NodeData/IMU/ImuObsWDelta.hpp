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
#include "Navigation/Transformations/Units.hpp"

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

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return ImuObs::GetStaticDescriptorCount() + 7; }

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
        if (idx < ImuObs::GetStaticDescriptorCount()) { return ImuObs::getValueAt(idx); }
        switch (idx)
        {
        case ImuObs::GetStaticDescriptorCount() + 0: // dTime [s]
            return dtime;
        case ImuObs::GetStaticDescriptorCount() + 1: // dTheta X [deg]
            return rad2deg(dtheta.x());
        case ImuObs::GetStaticDescriptorCount() + 2: // dTheta Y [deg]
            return rad2deg(dtheta.y());
        case ImuObs::GetStaticDescriptorCount() + 3: // dTheta Z [deg]
            return rad2deg(dtheta.z());
        case ImuObs::GetStaticDescriptorCount() + 4: // dVelocity X [m/s]
            return dvel.x();
        case ImuObs::GetStaticDescriptorCount() + 5: // dVelocity Y [m/s]
            return dvel.y();
        case ImuObs::GetStaticDescriptorCount() + 6: // dVelocity Z [m/s]
            return dvel.z();
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
        if (idx < ImuObs::GetStaticDescriptorCount()) { return ImuObs::setValueAt(idx, value); }
        switch (idx)
        {
        case ImuObs::GetStaticDescriptorCount() + 0: // dTime [s]
            dtime = value;
            break;
        case ImuObs::GetStaticDescriptorCount() + 1: // dTheta X [deg]
            dtheta.x() = deg2rad(value);
            break;
        case ImuObs::GetStaticDescriptorCount() + 2: // dTheta Y [deg]
            dtheta.y() = deg2rad(value);
            break;
        case ImuObs::GetStaticDescriptorCount() + 3: // dTheta Z [deg]
            dtheta.z() = deg2rad(value);
            break;
        case ImuObs::GetStaticDescriptorCount() + 4: // dVelocity X [m/s]
            dvel.x() = value;
            break;
        case ImuObs::GetStaticDescriptorCount() + 5: // dVelocity Y [m/s]
            dvel.y() = value;
            break;
        case ImuObs::GetStaticDescriptorCount() + 6: // dVelocity Z [m/s]
            dvel.z() = value;
            break;
        default:
            return false;
        }

        return true;
    }

    /// The time interval that the delta angle and velocities are integrated over in [seconds].
    double dtime = 0.0;
    /// The delta rotation angles in [rad] incurred due to rotation, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    Eigen::Vector3d dtheta;
    /// The delta velocity in [m/s] incurred due to motion, by the local platform reference frame,
    /// since the last time the values were outputted by the device.
    Eigen::Vector3d dvel;
};

} // namespace NAV
