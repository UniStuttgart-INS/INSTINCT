// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuObs.hpp
/// @brief Parent Class for all IMU Observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-12

#pragma once

#include "NodeData/NodeData.hpp"

#include "ImuPos.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// IMU Observation storage class
class ImuObs : public NodeData
{
  public:
    /// @brief Constructor
    /// @param[in] imuPos Reference to the position and rotation info of the Imu
    explicit ImuObs(const ImuPos& imuPos)
        : imuPos(imuPos) {}

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "ImuObs";
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
            "Time since startup [ns]",
            "Accel X [m/s^2]",
            "Accel Y [m/s^2]",
            "Accel Z [m/s^2]",
            "Gyro X [rad/s]",
            "Gyro Y [rad/s]",
            "Gyro Z [rad/s]",
            "Mag X [Gauss]",
            "Mag Y [Gauss]",
            "Mag Z [Gauss]",
            "Temperature [°C]",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 11; }

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
        case 0: // Time since startup [ns]
            if (timeSinceStartup.has_value()) { return static_cast<double>(timeSinceStartup.value()); }
            break;
        case 1: // Accel X [m/s^2]
            return p_acceleration.x();
        case 2: // Accel Y [m/s^2]
            return p_acceleration.y();
        case 3: // Accel Z [m/s^2]
            return p_acceleration.z();
        case 4: // Gyro X [rad/s]
            return p_angularRate.x();
        case 5: // Gyro Y [rad/s]
            return p_angularRate.y();
        case 6: // Gyro Z [rad/s]
            return p_angularRate.z();
        case 7: // Mag X [Gauss]
            if (p_magneticField.has_value()) { return p_magneticField->x(); }
            break;
        case 8: // Mag Y [Gauss]
            if (p_magneticField.has_value()) { return p_magneticField->y(); }
            break;
        case 9: // Mag Z [Gauss]
            if (p_magneticField.has_value()) { return p_magneticField->z(); }
            break;
        case 10: // Temperature [°C]
            return temperature;
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
        case 0: // Time since startup [ns]
            if (value >= 0.0)
            {
                timeSinceStartup = static_cast<size_t>(value);
                return true;
            }
            break;
        case 1: // Accel X [m/s^2]
            p_acceleration.x() = value;
            return true;
        case 2: // Accel Y [m/s^2]
            p_acceleration.y() = value;
            return true;
        case 3: // Accel Z [m/s^2]
            p_acceleration.z() = value;
            return true;
        case 4: // Gyro X [rad/s]
            p_angularRate.x() = value;
            return true;
        case 5: // Gyro Y [rad/s]
            p_angularRate.y() = value;
            return true;
        case 6: // Gyro Z [rad/s]
            p_angularRate.z() = value;
            return true;
        case 7: // Mag X [Gauss]
            if (p_magneticField.has_value())
            {
                p_magneticField->x() = value;
                return true;
            }
            break;
        case 8: // Mag Y [Gauss]
            if (p_magneticField.has_value())
            {
                p_magneticField->y() = value;
                return true;
            }
            break;
        case 9: // Mag Z [Gauss]
            if (p_magneticField.has_value())
            {
                p_magneticField->z() = value;
                return true;
            }
            break;
        case 10: // Temperature [°C]
            temperature = value;
            return true;
        default:
            return false;
        }

        return false;
    }

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;

    /// The system time since startup measured in [nano seconds].
    std::optional<uint64_t> timeSinceStartup;

    /// The IMU acceleration measured in units of [m/s^2], given in the platform frame.
    Eigen::Vector3d p_acceleration;
    /// The IMU angular rate measured in units of [rad/s], given in the platform frame.
    Eigen::Vector3d p_angularRate;

    /// The IMU magnetic field measured in units of [Gauss], given in the platform frame.
    std::optional<Eigen::Vector3d> p_magneticField;
    /// The IMU temperature measured in units of [Celsius].
    std::optional<double> temperature = 0.0;
};

} // namespace NAV
