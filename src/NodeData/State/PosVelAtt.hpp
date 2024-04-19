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

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVel::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Roll [deg]");
        desc.emplace_back("Pitch [deg]");
        desc.emplace_back("Yaw [deg]");
        desc.emplace_back("Quaternion::w");
        desc.emplace_back("Quaternion::x");
        desc.emplace_back("Quaternion::y");
        desc.emplace_back("Quaternion::z");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 22; }

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
            return PosVel::getValueAt(idx);
        case 15: // Roll [deg]
            return rad2deg(rollPitchYaw().x());
        case 16: // Pitch [deg]
            return rad2deg(rollPitchYaw().y());
        case 17: // Yaw [deg]
            return rad2deg(rollPitchYaw().z());
        case 18: // Quaternion::w
            return n_Quat_b().w();
        case 19: // Quaternion::x
            return n_Quat_b().x();
        case 20: // Quaternion::y
            return n_Quat_b().y();
        case 21: // Quaternion::z
            return n_Quat_b().z();
        default:
            return std::nullopt;
        }
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
