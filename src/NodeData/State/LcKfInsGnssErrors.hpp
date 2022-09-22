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
