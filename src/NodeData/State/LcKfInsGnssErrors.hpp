// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LcKfInsGnssErrors.hpp
/// @brief Loosely-coupled Kalman Filter INS/GNSS errors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-08

#pragma once

#include "util/Eigen.hpp"
#include "NodeData/NodeData.hpp"

#include "Navigation/Transformations/Units.hpp"

namespace NAV
{
/// Loosely-coupled Kalman Filter INS/GNSS errors
class LcKfInsGnssErrors : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "LcKfInsGnssErrors";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetDataDescriptors()
    {
        return {
            // PVAError
            "Roll error [deg]",
            "Pitch error [deg]",
            "Yaw error [deg]",
            "North velocity error [m/s]",
            "East velocity error [m/s]",
            "Down velocity error [m/s]",
            "Latitude error [deg]",
            "Longitude error [deg]",
            "Altitude error [m]",
            "Alpha_eb [deg]",
            "Beta_eb [deg]",
            "Gamma_eb [deg]",
            "ECEF X velocity error [m/s]",
            "ECEF Y velocity error [m/s]",
            "ECEF Z velocity error [m/s]",
            "ECEF X error [m]",
            "ECEF Y error [m]",
            "ECEF Z error [m]",
            // ImuBiases
            "Accelerometer bias b_X [m/s^2]",
            "Accelerometer bias b_Y [m/s^2]",
            "Accelerometer bias b_Z [m/s^2]",
            "Gyroscope bias b_X [rad/s]",
            "Gyroscope bias b_Y [rad/s]",
            "Gyroscope bias b_Z [rad/s]",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetDescriptorCount() { return 24; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> dataDescriptors() const override { return GetDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetDescriptorCount());
        if (frame == Frame::NED && 9 <= idx && idx <= 17) { return std::nullopt; }
        if (frame == Frame::ECEF && idx <= 8) { return std::nullopt; }

        switch (idx)
        {
        case 0: // Roll error [deg]
            return rad2deg(attitudeError(0));
        case 1: // Pitch error [deg]
            return rad2deg(attitudeError(1));
        case 2: // Yaw error [deg]
            return rad2deg(attitudeError(2));
        case 3: // North velocity error [m/s]
            return velocityError(0);
        case 4: // East velocity error [m/s]
            return velocityError(1);
        case 5: // Down velocity error [m/s]
            return velocityError(2);
        case 6: // Latitude error [deg]
            return rad2deg(positionError(0));
        case 7: // Longitude error [deg]
            return rad2deg(positionError(1));
        case 8: // Altitude error [m]
            return positionError(2);
        case 9: // Alpha_eb [deg]
            return rad2deg(attitudeError(0));
        case 10: // Beta_eb [deg]
            return rad2deg(attitudeError(1));
        case 11: // Gamma_eb [deg]
            return rad2deg(attitudeError(2));
        case 12: // ECEF X velocity error [m/s]
            return velocityError(0);
        case 13: // ECEF Y velocity error [m/s]
            return velocityError(1);
        case 14: // ECEF Z velocity error [m/s]
            return velocityError(2);
        case 15: // ECEF X error [m]
            return positionError(0);
        case 16: // ECEF Y error [m]
            return positionError(1);
        case 17: // ECEF Z error [m]
            return positionError(2);
        case 18: // Accelerometer bias b_X [m/s^2]
            return b_biasAccel(0);
        case 19: // Accelerometer bias b_Y [m/s^2]
            return b_biasAccel(1);
        case 20: // Accelerometer bias b_Z [m/s^2]
            return b_biasAccel(2);
        case 21: // Gyroscope bias b_X [rad/s]
            return b_biasGyro(0);
        case 22: // Gyroscope bias b_Y [rad/s]
            return b_biasGyro(1);
        case 23: // Gyroscope bias b_Z [rad/s]
            return b_biasGyro(2);
        default:
            return std::nullopt;
        }
    }

    /// @brief Available Frames
    enum class Frame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };
    /// Frame in which the errors are set
    Frame frame = Frame::NED;

    /// δ𝛙_{i,e,n}b_{i,e,n} The attitude error in {i,e,n} frame coordinates in [rad]
    Eigen::Vector3d attitudeError{ 0, 0, 0 };
    /// δ𝐯_{i,e,n} The velocity error in {i,e,n} coordinates in [m/s]
    Eigen::Vector3d velocityError{ 0, 0, 0 };
    /// NED:    δ𝐩 = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    /// ECEF/i: δr The position error in [m]
    Eigen::Vector3d positionError{ 0, 0, 0 };

    /// 𝐛_a The accelerometer bias in body frame in [m/s^2]
    Eigen::Vector3d b_biasAccel{ 0, 0, 0 };
    /// 𝐛_g The gyroscope bias in body frame in [rad/s]
    Eigen::Vector3d b_biasGyro{ 0, 0, 0 };
};

} // namespace NAV
