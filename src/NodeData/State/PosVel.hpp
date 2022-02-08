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
        _n_velocity = quaternion_ne() * e_velocity;
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
