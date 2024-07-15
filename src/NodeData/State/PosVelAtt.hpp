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

#include "Navigation/Transformations/Units.hpp"
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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 46; }

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
            return PosVel::getValueAt(idx);
        case 39: // Roll [deg]
            return rad2deg(rollPitchYaw().x());
        case 40: // Pitch [deg]
            return rad2deg(rollPitchYaw().y());
        case 41: // Yaw [deg]
            return rad2deg(rollPitchYaw().z());
        case 42: // Quaternion::w
            return n_Quat_b().w();
        case 43: // Quaternion::x
            return n_Quat_b().x();
        case 44: // Quaternion::y
            return n_Quat_b().y();
        case 45: // Quaternion::z
            return n_Quat_b().z();
        default:
            return std::nullopt;
        }
        return std::nullopt;
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

    /// @brief Set the State and the covariances
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_positionCovarianceMatrix Standard deviation of Position in ECEF coordinates [m]
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    void setStateAndCovariance_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_positionCovarianceMatrix,
                                 const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityCovarianceMatrix,
                                 const Eigen::Quaterniond& e_Quat_b)
    {
        setPositionAndStdDev_e(e_position, e_positionCovarianceMatrix);
        setVelocityAndStdDev_e(e_velocity, e_velocityCovarianceMatrix);
        setAttitude_e_Quat_b(e_Quat_b);
    }

    /// @brief Set the State and the covariances
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_positionCovarianceMatrix Standard deviation of Position in NED coordinates [m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_velocityCovarianceMatrix Covariance matrix of Velocity in NED coordinates [m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    void setStateAndCovariance_n(const Eigen::Vector3d& lla_position, const Eigen::Matrix3d& n_positionCovarianceMatrix,
                                 const Eigen::Vector3d& n_velocity, const Eigen::Matrix3d& n_velocityCovarianceMatrix,
                                 const Eigen::Quaterniond& n_Quat_b)
    {
        setPositionAndStdDev_lla(lla_position, n_positionCovarianceMatrix);
        setVelocityAndStdDev_n(n_velocity, n_velocityCovarianceMatrix);
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
