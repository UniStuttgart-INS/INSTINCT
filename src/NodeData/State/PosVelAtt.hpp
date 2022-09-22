// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PosVelAtt.hpp
/// @brief Position, Velocity and Attitude Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "util/Eigen.hpp"
#include "NodeData/State/PosVel.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class PosVelAtt : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "PosVelAtt";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] const Eigen::Quaterniond& n_Quat_b() const
    {
        return _n_Quat_b;
    }

    /// @brief Returns the Quaternion from navigation to body frame (NED)
    /// @return The Quaternion for the rotation from navigation to body coordinates
    [[nodiscard]] Eigen::Quaterniond b_Quat_n() const
    {
        return n_Quat_b().conjugate();
    }

    /// @brief Returns the Quaternion from body to Earth-fixed frame
    /// @return The Quaternion for the rotation from body to earth coordinates
    [[nodiscard]] const Eigen::Quaterniond& e_Quat_b() const
    {
        return _e_Quat_b;
    }

    /// @brief Returns the Quaternion from Earth-fixed to body frame
    /// @return The Quaternion for the rotation from earth to body coordinates
    [[nodiscard]] Eigen::Quaterniond b_Quat_e() const
    {
        return e_Quat_b().conjugate();
    }

    /// @brief Returns the Roll, Pitch and Yaw angles in [rad]
    /// @return [roll, pitch, yaw]^T
    [[nodiscard]] Eigen::Vector3d rollPitchYaw() const
    {
        return trafo::quat2eulerZYX(n_Quat_b());
    }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Quaternion from body to earth frame
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    void setAttitude_e_Quat_b(const Eigen::Quaterniond& e_Quat_b)
    {
        _e_Quat_b = e_Quat_b;
        _n_Quat_b = n_Quat_e() * e_Quat_b;
    }

    /// @brief Set the Quaternion from body to navigation frame
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    void setAttitude_n_Quat_b(const Eigen::Quaterniond& n_Quat_b)
    {
        _e_Quat_b = e_Quat_n() * n_Quat_b;
        _n_Quat_b = n_Quat_b;
    }

    /// @brief Set the State
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    void setState_e(const Eigen::Vector3d& e_position, const Eigen::Vector3d& e_velocity, const Eigen::Quaterniond& e_Quat_b)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
        setAttitude_e_Quat_b(e_Quat_b);
    }

    /// @brief Set the State
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    void setState_n(const Eigen::Vector3d& lla_position, const Eigen::Vector3d& n_velocity, const Eigen::Quaterniond& n_Quat_b)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
        setAttitude_n_Quat_b(n_Quat_b);
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Quaternion body to earth frame
    Eigen::Quaterniond _e_Quat_b{ 0, 0, 0, 0 };
    /// Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond _n_Quat_b{ 0, 0, 0, 0 };
};

} // namespace NAV
