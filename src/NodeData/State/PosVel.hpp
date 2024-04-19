// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PosVel.hpp
/// @brief Position, Velocity and Attitude Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-27

#pragma once

#include "NodeData/State/Pos.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class PosVel : public Pos
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "PosVel";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = Pos::parentTypes();
        parent.push_back(Pos::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = Pos::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Velocity norm [m/s]");
        desc.emplace_back("X velocity ECEF [m/s]");
        desc.emplace_back("Y velocity ECEF [m/s]");
        desc.emplace_back("Z velocity ECEF [m/s]");
        desc.emplace_back("North velocity [m/s]");
        desc.emplace_back("East velocity [m/s]");
        desc.emplace_back("Down velocity [m/s]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 15; }

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
        case 0: // Latitude [deg]
        case 1: // Longitude [deg]
        case 2: // Altitude [m]
        case 3: // North/South [m]
        case 4: // East/West [m]
        case 5: // X-ECEF [m]
        case 6: // Y-ECEF [m]
        case 7: // Z-ECEF [m]
            return Pos::getValueAt(idx);
        case 8: // Velocity norm [m/s]
            return e_velocity().norm();
        case 9: // X velocity ECEF [m/s]
            return e_velocity().x();
        case 10: // Y velocity ECEF [m/s]
            return e_velocity().y();
        case 11: // Z velocity ECEF [m/s]
            return e_velocity().z();
        case 12: // North velocity [m/s]
            return n_velocity().x();
        case 13: // East velocity [m/s]
            return n_velocity().y();
        case 14: // Down velocity [m/s]
            return n_velocity().z();
        default:
            return std::nullopt;
        }
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocity() const { return _e_velocity; }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocity() const { return _n_velocity; }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Velocity in the earth frame
    /// @param[in] e_velocity The new velocity in the earth frame
    void setVelocity_e(const Eigen::Vector3d& e_velocity)
    {
        _e_velocity = e_velocity;
        _n_velocity = n_Quat_e() * e_velocity;
    }

    /// @brief Set the Velocity in the NED frame
    /// @param[in] n_velocity The new velocity in the NED frame
    void setVelocity_n(const Eigen::Vector3d& n_velocity)
    {
        _e_velocity = e_Quat_n() * n_velocity;
        _n_velocity = n_velocity;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocity{ std::nan(""), std::nan(""), std::nan("") };
    /// Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocity{ std::nan(""), std::nan(""), std::nan("") };
};

} // namespace NAV
