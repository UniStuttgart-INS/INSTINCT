// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InsGnssLCKFSolution.hpp
/// @brief Loosely-coupled Kalman Filter INS/GNSS errors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-08

#pragma once

#include "util/Eigen.hpp"
#include "PosVelAtt.hpp"

#include "Navigation/Transformations/Units.hpp"

namespace NAV
{
/// Loosely-coupled Kalman Filter INS/GNSS errors
class InsGnssLCKFSolution : public PosVelAtt
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "InsGnssLCKFSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVelAtt::parentTypes();
        parent.push_back(PosVelAtt::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVelAtt::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Roll error [deg]");
        desc.emplace_back("Pitch error [deg]");
        desc.emplace_back("Yaw error [deg]");
        desc.emplace_back("North velocity error [m/s]");
        desc.emplace_back("East velocity error [m/s]");
        desc.emplace_back("Down velocity error [m/s]");
        desc.emplace_back("Latitude error [deg]");
        desc.emplace_back("Longitude error [deg]");
        desc.emplace_back("Altitude error [m]");
        desc.emplace_back("Alpha_eb [deg]");
        desc.emplace_back("Beta_eb [deg]");
        desc.emplace_back("Gamma_eb [deg]");
        desc.emplace_back("ECEF X velocity error [m/s]");
        desc.emplace_back("ECEF Y velocity error [m/s]");
        desc.emplace_back("ECEF Z velocity error [m/s]");
        desc.emplace_back("ECEF X error [m]");
        desc.emplace_back("ECEF Y error [m]");
        desc.emplace_back("ECEF Z error [m]");
        desc.emplace_back("Accelerometer bias b_X [m/s^2]");
        desc.emplace_back("Accelerometer bias b_Y [m/s^2]");
        desc.emplace_back("Accelerometer bias b_Z [m/s^2]");
        desc.emplace_back("Gyroscope bias b_X [rad/s]");
        desc.emplace_back("Gyroscope bias b_Y [rad/s]");
        desc.emplace_back("Gyroscope bias b_Z [rad/s]");
        return desc;
    }

    /// @brief Get the number of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 46; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (frame == Frame::NED && 31 <= idx && idx <= 39) { return std::nullopt; }
        if (frame == Frame::ECEF && 22 <= idx && idx <= 30) { return std::nullopt; }

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
        case 8:  // Velocity norm [m/s]
        case 9:  // X velocity ECEF [m/s]
        case 10: // Y velocity ECEF [m/s]
        case 11: // Z velocity ECEF [m/s]
        case 12: // North velocity [m/s]
        case 13: // East velocity [m/s]
        case 14: // Down velocity [m/s]
        case 15: // Roll [deg]
        case 16: // Pitch [deg]
        case 17: // Yaw [deg]
        case 18: // Quaternion::w
        case 19: // Quaternion::x
        case 20: // Quaternion::y
        case 21: // Quaternion::z
            return PosVelAtt::getValueAt(idx);
        case 22: // Roll error [deg]
            return rad2deg(attitudeError(0));
        case 23: // Pitch error [deg]
            return rad2deg(attitudeError(1));
        case 24: // Yaw error [deg]
            return rad2deg(attitudeError(2));
        case 25: // North velocity error [m/s]
            return velocityError(0);
        case 26: // East velocity error [m/s]
            return velocityError(1);
        case 27: // Down velocity error [m/s]
            return velocityError(2);
        case 28: // Latitude error [deg]
            return rad2deg(positionError(0));
        case 29: // Longitude error [deg]
            return rad2deg(positionError(1));
        case 30: // Altitude error [m]
            return positionError(2);
        case 31: // Alpha_eb [deg]
            return rad2deg(attitudeError(0));
        case 32: // Beta_eb [deg]
            return rad2deg(attitudeError(1));
        case 33: // Gamma_eb [deg]
            return rad2deg(attitudeError(2));
        case 34: // ECEF X velocity error [m/s]
            return velocityError(0);
        case 35: // ECEF Y velocity error [m/s]
            return velocityError(1);
        case 36: // ECEF Z velocity error [m/s]
            return velocityError(2);
        case 37: // ECEF X error [m]
            return positionError(0);
        case 38: // ECEF Y error [m]
            return positionError(1);
        case 39: // ECEF Z error [m]
            return positionError(2);
        case 40: // Accelerometer bias b_X [m/s^2]
            return b_biasAccel(0);
        case 41: // Accelerometer bias b_Y [m/s^2]
            return b_biasAccel(1);
        case 42: // Accelerometer bias b_Z [m/s^2]
            return b_biasAccel(2);
        case 43: // Gyroscope bias b_X [rad/s]
            return b_biasGyro(0);
        case 44: // Gyroscope bias b_Y [rad/s]
            return b_biasGyro(1);
        case 45: // Gyroscope bias b_Z [rad/s]
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
