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
#include <cstdint>

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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVelAtt::GetStaticDescriptorCount() + 24; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (idx < PosVelAtt::GetStaticDescriptorCount()) { return PosVelAtt::getValueAt(idx); }
        switch (idx)
        {
        case PosVelAtt::GetStaticDescriptorCount() + 0: // KF State Roll error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(0)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 1: // KF State Pitch error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(1)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 2: // KF State Yaw error [deg]
            if (frame == Frame::NED) { return rad2deg(attitudeError(2)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 3: // KF State Position North error [deg]
            if (frame == Frame::NED) { return rad2deg(positionError(0)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 4: // KF State Position East error [deg]
            if (frame == Frame::NED) { return rad2deg(positionError(1)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 5: // KF State Position Down error [m]
            if (frame == Frame::NED) { return positionError(2); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 6: // KF State Velocity North error [m/s]
            if (frame == Frame::NED) { return velocityError(0); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 7: // KF State Velocity East error [m/s]
            if (frame == Frame::NED) { return velocityError(1); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 8: // KF State Velocity Down error [m/s]
            if (frame == Frame::NED) { return velocityError(2); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 9: // KF State Alpha_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(0)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 10: // KF State Beta_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(1)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 11: // KF State Gamma_eb [deg]
            if (frame == Frame::ECEF) { return rad2deg(attitudeError(2)); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 12: // KF State Position ECEF X error [m]
            if (frame == Frame::ECEF) { return positionError(0); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 13: // KF State Position ECEF Y error [m]
            if (frame == Frame::ECEF) { return positionError(1); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 14: // KF State Position ECEF Z error [m]
            if (frame == Frame::ECEF) { return positionError(2); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 15: // KF State Velocity ECEF X error [m/s]
            if (frame == Frame::ECEF) { return velocityError(0); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 16: // KF State Velocity ECEF Y error [m/s]
            if (frame == Frame::ECEF) { return velocityError(1); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 17: // KF State Velocity ECEF Z error [m/s]
            if (frame == Frame::ECEF) { return velocityError(2); }
            break;
        case PosVelAtt::GetStaticDescriptorCount() + 18: // Accelerometer bias b_X [m/s^2]
            return b_biasAccel(0);
        case PosVelAtt::GetStaticDescriptorCount() + 19: // Accelerometer bias b_Y [m/s^2]
            return b_biasAccel(1);
        case PosVelAtt::GetStaticDescriptorCount() + 20: // Accelerometer bias b_Z [m/s^2]
            return b_biasAccel(2);
        case PosVelAtt::GetStaticDescriptorCount() + 21: // Gyroscope bias b_X [rad/s]
            return b_biasGyro(0);
        case PosVelAtt::GetStaticDescriptorCount() + 22: // Gyroscope bias b_Y [rad/s]
            return b_biasGyro(1);
        case PosVelAtt::GetStaticDescriptorCount() + 23: // Gyroscope bias b_Z [rad/s]
            return b_biasGyro(2);
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Available Frames
    enum class Frame : uint8_t
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
