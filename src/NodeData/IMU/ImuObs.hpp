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
            "Mag uncomp X [Gauss]",
            "Mag uncomp Y [Gauss]",
            "Mag uncomp Z [Gauss]",
            "Accel uncomp X [m/s^2]",
            "Accel uncomp Y [m/s^2]",
            "Accel uncomp Z [m/s^2]",
            "Gyro uncomp X [rad/s]",
            "Gyro uncomp Y [rad/s]",
            "Gyro uncomp Z [rad/s]",
            "Mag Comp X [Gauss]",
            "Mag Comp Y [Gauss]",
            "Mag Comp Z [Gauss]",
            "Accel Comp X [m/s^2]",
            "Accel Comp Y [m/s^2]",
            "Accel Comp Z [m/s^2]",
            "Gyro Comp X [rad/s]",
            "Gyro Comp Y [rad/s]",
            "Gyro Comp Z [rad/s]",
            "Temperature [°C]",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 20; }

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
        case 1: // Mag uncomp X [Gauss]
            if (magUncompXYZ.has_value()) { return magUncompXYZ->x(); }
            break;
        case 2: // Mag uncomp Y [Gauss]
            if (magUncompXYZ.has_value()) { return magUncompXYZ->y(); }
            break;
        case 3: // Mag uncomp Z [Gauss]
            if (magUncompXYZ.has_value()) { return magUncompXYZ->z(); }
            break;
        case 4: // Accel uncomp X [m/s^2]
            if (accelUncompXYZ.has_value()) { return accelUncompXYZ->x(); }
            break;
        case 5: // Accel uncomp Y [m/s^2]
            if (accelUncompXYZ.has_value()) { return accelUncompXYZ->y(); }
            break;
        case 6: // Accel uncomp Z [m/s^2]
            if (accelUncompXYZ.has_value()) { return accelUncompXYZ->z(); }
            break;
        case 7: // Gyro uncomp X [rad/s]
            if (gyroUncompXYZ.has_value()) { return gyroUncompXYZ->x(); }
            break;
        case 8: // Gyro uncomp Y [rad/s]
            if (gyroUncompXYZ.has_value()) { return gyroUncompXYZ->y(); }
            break;
        case 9: // Gyro uncomp Z [rad/s]
            if (gyroUncompXYZ.has_value()) { return gyroUncompXYZ->z(); }
            break;
        case 10: // Mag Comp X [Gauss]
            if (magCompXYZ.has_value()) { return magCompXYZ->x(); }
            break;
        case 11: // Mag Comp Y [Gauss]
            if (magCompXYZ.has_value()) { return magCompXYZ->y(); }
            break;
        case 12: // Mag Comp Z [Gauss]
            if (magCompXYZ.has_value()) { return magCompXYZ->z(); }
            break;
        case 13: // Accel Comp X [m/s^2]
            if (accelCompXYZ.has_value()) { return accelCompXYZ->x(); }
            break;
        case 14: // Accel Comp Y [m/s^2]
            if (accelCompXYZ.has_value()) { return accelCompXYZ->y(); }
            break;
        case 15: // Accel Comp Z [m/s^2]
            if (accelCompXYZ.has_value()) { return accelCompXYZ->z(); }
            break;
        case 16: // Gyro Comp X [rad/s]
            if (gyroCompXYZ.has_value()) { return gyroCompXYZ->x(); }
            break;
        case 17: // Gyro Comp Y [rad/s]
            if (gyroCompXYZ.has_value()) { return gyroCompXYZ->y(); }
            break;
        case 18: // Gyro Comp Z [rad/s]
            if (gyroCompXYZ.has_value()) { return gyroCompXYZ->z(); }
            break;
        case 19: // Temperature [°C]
            if (temperature.has_value()) { return temperature.value(); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// Position and rotation information for conversion from platform to body frame
    const ImuPos& imuPos;

    /// The system time since startup measured in [nano seconds].
    std::optional<uint64_t> timeSinceStartup;

    /// The IMU magnetic field measured in units of [Gauss], given in the platform frame.
    std::optional<Eigen::Vector3d> magUncompXYZ;
    /// The IMU acceleration measured in units of [m/s^2], given in the platform frame.
    std::optional<Eigen::Vector3d> accelUncompXYZ;
    /// The IMU angular rate measured in units of [rad/s], given in the platform frame.
    std::optional<Eigen::Vector3d> gyroUncompXYZ;

    /// The compensated magnetic field measured in units of [Gauss], and given in the platform frame.
    std::optional<Eigen::Vector3d> magCompXYZ;
    /// The compensated acceleration measured in units of [m/s^2], and given in the platform frame.
    std::optional<Eigen::Vector3d> accelCompXYZ;
    /// The compensated angular rate measured in units of [rad/s], and given in the platform frame.
    std::optional<Eigen::Vector3d> gyroCompXYZ;

    /// The IMU temperature measured in units of [Celsius].
    std::optional<double> temperature = 0.0;
};

} // namespace NAV
