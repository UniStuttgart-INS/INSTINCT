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
        desc.emplace_back("KF State Roll error [deg]");
        desc.emplace_back("KF State Pitch error [deg]");
        desc.emplace_back("KF State Yaw error [deg]");
        desc.emplace_back("KF State Position North error [deg]");
        desc.emplace_back("KF State Position East error [deg]");
        desc.emplace_back("KF State Position Down error [m]");
        desc.emplace_back("KF State Velocity North error [m/s]");
        desc.emplace_back("KF State Velocity East error [m/s]");
        desc.emplace_back("KF State Velocity Down error [m/s]");
        desc.emplace_back("KF State Alpha_eb [deg]");
        desc.emplace_back("KF State Beta_eb [deg]");
        desc.emplace_back("KF State Gamma_eb [deg]");
        desc.emplace_back("KF State Position ECEF X error [m]");
        desc.emplace_back("KF State Position ECEF Y error [m]");
        desc.emplace_back("KF State Position ECEF Z error [m]");
        desc.emplace_back("KF State Velocity ECEF X error [m/s]");
        desc.emplace_back("KF State Velocity ECEF Y error [m/s]");
        desc.emplace_back("KF State Velocity ECEF Z error [m/s]");
        desc.emplace_back("Accelerometer bias b_X [m/s^2]");
        desc.emplace_back("Accelerometer bias b_Y [m/s^2]");
        desc.emplace_back("Accelerometer bias b_Z [m/s^2]");
        desc.emplace_back("Gyroscope bias b_X [rad/s]");
        desc.emplace_back("Gyroscope bias b_Y [rad/s]");
        desc.emplace_back("Gyroscope bias b_Z [rad/s]");
        return desc;
    }

    /// @brief Get the number of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 70; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

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
            return PosVelAtt::getValueAt(idx);
        case 46: // KF State Roll error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(0)); }
            break;
        case 47: // KF State Pitch error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(1)); }
            break;
        case 48: // KF State Yaw error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(2)); }
            break;
        case 49: // KF State Position North error [deg]
            if (frame == Frame::NED) { return rad2deg(positionError(0)); }
            break;
        case 50: // KF State Position East error [deg]
            if (frame == Frame::NED) { return rad2deg(positionError(1)); }
            break;
        case 51: // KF State Position Down error [m]
            if (frame == Frame::NED) { return positionError(2); }
            break;
        case 52: // KF State Velocity North error [m/s]
            if (frame == Frame::NED) { return velocityError(0); }
            break;
        case 53: // KF State Velocity East error [m/s]
            if (frame == Frame::NED) { return velocityError(1); }
            break;
        case 54: // KF State Velocity Down error [m/s]
            if (frame == Frame::NED) { return velocityError(2); }
            break;
        case 55: // KF State Alpha_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(0)); }
            break;
        case 56: // KF State Beta_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(1)); }
            break;
        case 57: // KF State Gamma_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(2)); }
            break;
        case 58: // KF State Position ECEF X error [m]
            if (frame == Frame::ECEF) { return positionError(0); }
            break;
        case 59: // KF State Position ECEF Y error [m]
            if (frame == Frame::ECEF) { return positionError(1); }
            break;
        case 60: // KF State Position ECEF Z error [m]
            if (frame == Frame::ECEF) { return positionError(2); }
            break;
        case 61: // KF State Velocity ECEF X error [m/s]
            if (frame == Frame::ECEF) { return velocityError(0); }
            break;
        case 62: // KF State Velocity ECEF Y error [m/s]
            if (frame == Frame::ECEF) { return velocityError(1); }
            break;
        case 63: // KF State Velocity ECEF Z error [m/s]
            if (frame == Frame::ECEF) { return velocityError(2); }
            break;
        case 64: // Accelerometer bias b_X [m/s^2]
            return b_biasAccel(0);
        case 65: // Accelerometer bias b_Y [m/s^2]
            return b_biasAccel(1);
        case 66: // Accelerometer bias b_Z [m/s^2]
            return b_biasAccel(2);
        case 67: // Gyroscope bias b_X [rad/s]
            return b_biasGyro(0);
        case 68: // Gyroscope bias b_Y [rad/s]
            return b_biasGyro(1);
        case 69: // Gyroscope bias b_Z [rad/s]
            return b_biasGyro(2);
        default:
            return std::nullopt;
        }
        return std::nullopt;
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
