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
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVel::GetStaticDescriptorCount() + 7; }

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
        if (idx < PosVel::GetStaticDescriptorCount()) { return PosVel::getValueAt(idx); }
        switch (idx)
        {
        case PosVel::GetStaticDescriptorCount() + 0: // Roll [deg]
            if (_e_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().x()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 1: // Pitch [deg]
            if (_e_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().y()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 2: // Yaw [deg]
            if (_e_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().z()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 3: // Quaternion::w
            if (_e_Quat_b.norm() != 0) { return n_Quat_b().w(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 4: // Quaternion::x
            if (_e_Quat_b.norm() != 0) { return n_Quat_b().x(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 5: // Quaternion::y
            if (_e_Quat_b.norm() != 0) { return n_Quat_b().y(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 6: // Quaternion::z
            if (_e_Quat_b.norm() != 0) { return n_Quat_b().z(); }
            break;
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
    template<typename Derived>
    void setAttitude_e_Quat_b(const Eigen::QuaternionBase<Derived>& e_Quat_b)
    {
        _e_Quat_b = e_Quat_b;
        _n_Quat_b = n_Quat_e() * e_Quat_b;
    }

    /// @brief Set the Quaternion from body to navigation frame
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    template<typename Derived>
    void setAttitude_n_Quat_b(const Eigen::QuaternionBase<Derived>& n_Quat_b)
    {
        _e_Quat_b = e_Quat_n() * n_Quat_b;
        _n_Quat_b = n_Quat_b;
    }

    /// @brief Set the State
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    template<typename DerivedP, typename DerivedV, typename DerivedA>
    void setState_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedV>& e_velocity, const Eigen::QuaternionBase<DerivedA>& e_Quat_b)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
        setAttitude_e_Quat_b(e_Quat_b);
    }

    /// @brief Set the State
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    template<typename DerivedP, typename DerivedV, typename DerivedA>
    void setState_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedV>& n_velocity, const Eigen::QuaternionBase<DerivedA>& n_Quat_b)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
        setAttitude_n_Quat_b(n_Quat_b);
    }

    /// @brief Set the State and the standard deviations
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_positionCovarianceMatrix Standard deviation of Position in ECEF coordinates [m]
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    template<typename DerivedP, typename DerivedP2, typename DerivedV, typename DerivedV2, typename DerivedA>
    void setStateAndStdDev_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedP2>& e_positionCovarianceMatrix,
                             const Eigen::MatrixBase<DerivedV>& e_velocity, const Eigen::MatrixBase<DerivedV2>& e_velocityCovarianceMatrix,
                             const Eigen::QuaternionBase<DerivedA>& e_Quat_b)
    {
        setPositionAndStdDev_e(e_position, e_positionCovarianceMatrix);
        setVelocityAndStdDev_e(e_velocity, e_velocityCovarianceMatrix);
        setAttitude_e_Quat_b(e_Quat_b);
    }

    /// @brief Set the State and the standard deviations
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_positionCovarianceMatrix Standard deviation of Position in NED coordinates [m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_velocityCovarianceMatrix Covariance matrix of Velocity in NED coordinates [m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    template<typename DerivedP, typename DerivedP2, typename DerivedV, typename DerivedV2, typename DerivedA>
    void setStateAndStdDev_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedP2>& n_positionCovarianceMatrix,
                             const Eigen::MatrixBase<DerivedV>& n_velocity, const Eigen::MatrixBase<DerivedV2>& n_velocityCovarianceMatrix,
                             const Eigen::QuaternionBase<DerivedA>& n_Quat_b)
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
