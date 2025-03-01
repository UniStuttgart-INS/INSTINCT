// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PosVel.hpp
/// @brief Position and Velocity Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-27

#pragma once

#include "NodeData/State/Pos.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace NAV
{
/// Position and Velocity Storage Class
class PosVel : public Pos
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "PosVel";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

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
        desc.emplace_back("X velocity ECEF StDev [m/s]");
        desc.emplace_back("Y velocity ECEF StDev [m/s]");
        desc.emplace_back("Z velocity ECEF StDev [m/s]");
        desc.emplace_back("XY velocity StDev [m]");
        desc.emplace_back("XZ velocity StDev [m]");
        desc.emplace_back("YZ velocity StDev [m]");
        desc.emplace_back("North velocity StDev [m/s]");
        desc.emplace_back("East velocity StDev [m/s]");
        desc.emplace_back("Down velocity StDev [m/s]");
        desc.emplace_back("NE velocity StDev [m]");
        desc.emplace_back("ND velocity StDev [m]");
        desc.emplace_back("ED velocity StDev [m]");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return Pos::GetStaticDescriptorCount() + 19; }

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
        if (idx < Pos::GetStaticDescriptorCount()) { return Pos::getValueAt(idx); }
        switch (idx)
        {
        case Pos::GetStaticDescriptorCount() + 0: // Velocity norm [m/s]
            return e_velocity().norm();
        case Pos::GetStaticDescriptorCount() + 1: // X velocity ECEF [m/s]
            return e_velocity().x();
        case Pos::GetStaticDescriptorCount() + 2: // Y velocity ECEF [m/s]
            return e_velocity().y();
        case Pos::GetStaticDescriptorCount() + 3: // Z velocity ECEF [m/s]
            return e_velocity().z();
        case Pos::GetStaticDescriptorCount() + 4: // North velocity [m/s]
            return n_velocity().x();
        case Pos::GetStaticDescriptorCount() + 5: // East velocity [m/s]
            return n_velocity().y();
        case Pos::GetStaticDescriptorCount() + 6: // Down velocity [m/s]
            return n_velocity().z();
        case Pos::GetStaticDescriptorCount() + 7: // X velocity ECEF StDev [m/s]
            if (auto stDev = e_velocityStdev()) { return stDev->x(); }
            break;
        case Pos::GetStaticDescriptorCount() + 8: // Y velocity ECEF StDev [m/s]
            if (auto stDev = e_velocityStdev()) { return stDev->y(); }
            break;
        case Pos::GetStaticDescriptorCount() + 9: // Z velocity ECEF StDev [m/s]
            if (auto stDev = e_velocityStdev()) { return stDev->z(); }
            break;
        case Pos::GetStaticDescriptorCount() + 10: // XY velocity StDev [m]
            if (_e_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::VelX, Keys::VelY))); }
            break;
        case Pos::GetStaticDescriptorCount() + 11: // XZ velocity StDev [m]
            if (_e_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::VelX, Keys::VelZ))); }
            break;
        case Pos::GetStaticDescriptorCount() + 12: // YZ velocity StDev [m]
            if (_e_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::VelY, Keys::VelZ))); }
            break;
        case Pos::GetStaticDescriptorCount() + 13: // North velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->x(); }
            break;
        case Pos::GetStaticDescriptorCount() + 14: // East velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->y(); }
            break;
        case Pos::GetStaticDescriptorCount() + 15: // Down velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->z(); }
            break;
        case Pos::GetStaticDescriptorCount() + 16: // NE velocity StDev [m]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::VelX, Keys::VelY))); }
            break;
        case Pos::GetStaticDescriptorCount() + 17: // ND velocity StDev [m]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::VelX, Keys::VelZ))); }
            break;
        case Pos::GetStaticDescriptorCount() + 18: // ED velocity StDev [m]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>)) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::VelY, Keys::VelZ))); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Set the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @param value Value to set
    /// @return True if the value was updated
    [[nodiscard]] bool setValueAt(size_t idx, double value) override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (idx < Pos::GetStaticDescriptorCount()) { return Pos::setValueAt(idx, value); }
        switch (idx)
        {
        case Pos::GetStaticDescriptorCount() + 0: // Velocity norm [m/s]
            _e_velocity = value * _e_velocity.normalized();
            _n_velocity = value * _n_velocity.normalized();
            break;
        case Pos::GetStaticDescriptorCount() + 1: // X velocity ECEF [m/s]
            _e_velocity(0) = value;
            _n_velocity = n_Quat_e() * _e_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 2: // Y velocity ECEF [m/s]
            _e_velocity(1) = value;
            _n_velocity = n_Quat_e() * _e_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 3: // Z velocity ECEF [m/s]
            _e_velocity(2) = value;
            _n_velocity = n_Quat_e() * _e_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 4: // North velocity [m/s]
            _n_velocity(0) = value;
            _e_velocity = e_Quat_n() * _n_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 5: // East velocity [m/s]
            _n_velocity(1) = value;
            _e_velocity = e_Quat_n() * _n_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 6: // Down velocity [m/s]
            _n_velocity(2) = value;
            _e_velocity = e_Quat_n() * _n_velocity;
            break;
        case Pos::GetStaticDescriptorCount() + 7:  // X velocity ECEF StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 8:  // Y velocity ECEF StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 9:  // Z velocity ECEF StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 10: // XY velocity StDev [m]
        case Pos::GetStaticDescriptorCount() + 11: // XZ velocity StDev [m]
        case Pos::GetStaticDescriptorCount() + 12: // YZ velocity StDev [m]
        case Pos::GetStaticDescriptorCount() + 13: // North velocity StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 14: // East velocity StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 15: // Down velocity StDev [m/s]
        case Pos::GetStaticDescriptorCount() + 16: // NE velocity StDev [m]
        case Pos::GetStaticDescriptorCount() + 17: // ND velocity StDev [m]
        case Pos::GetStaticDescriptorCount() + 18: // ED velocity StDev [m]
        default:
            return false;
        }

        return true;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocity() const { return _e_velocity; }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocity() const { return _n_velocity; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] std::optional<Eigen::Vector3d> e_velocityStdev() const
    {
        if (_e_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>))
        {
            return (*_e_covarianceMatrix)(Keys::Vel<Keys::MotionModelKey>, Keys::Vel<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
    }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] std::optional<Eigen::Vector3d> n_velocityStdev() const
    {
        if (_n_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Vel<Keys::MotionModelKey>))
        {
            return (*_n_covarianceMatrix)(Keys::Vel<Keys::MotionModelKey>, Keys::Vel<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
    }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Velocity in the earth frame
    /// @param[in] e_velocity The new velocity in the earth frame
    template<typename Derived>
    void setVelocity_e(const Eigen::MatrixBase<Derived>& e_velocity)
    {
        _e_velocity = e_velocity;
        _n_velocity = n_Quat_e() * e_velocity;
    }

    /// @brief Set the Velocity in the NED frame
    /// @param[in] n_velocity The new velocity in the NED frame
    template<typename Derived>
    void setVelocity_n(const Eigen::MatrixBase<Derived>& n_velocity)
    {
        _e_velocity = e_Quat_n() * n_velocity;
        _n_velocity = n_velocity;
    }

    /// @brief Set the position and velocity
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    template<typename DerivedP, typename DerivedV>
    void setPosVel_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedV>& e_velocity)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
    }

    /// @brief Set the position and velocity
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    template<typename DerivedP, typename DerivedV>
    void setPosVel_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedV>& n_velocity)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
    }

    /// @brief Set the position, velocity and the covariance matrix
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_covarianceMatrix 6x6 PosVel Error variance
    template<typename DerivedP, typename DerivedV, typename Derived>
    void setPosVelAndCov_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedV>& e_velocity,
                           const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
        setPosVelCovarianceMatrix_e(e_covarianceMatrix);
    }

    /// @brief Set the position, velocity and the covariance matrix
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_covarianceMatrix 6x6 PosVel Error variance
    template<typename DerivedP, typename DerivedV, typename Derived>
    void setPosVelAndCov_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedV>& n_velocity,
                           const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
        setPosVelCovarianceMatrix_n(n_covarianceMatrix);
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix 6x6 PosVel Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelCovarianceMatrix_e(const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.rows() == 6, "This function needs a 6x6 matrix as input");
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.cols() == 6, "This function needs a 6x6 matrix as input");

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            e_covarianceMatrix, Keys::PosVel<Keys::MotionModelKey>);

        Eigen::Quaterniond n_q_e = n_Quat_e();
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
        J.block<3, 3>(0, 0) = n_q_e.toRotationMatrix();
        J.block<3, 3>(3, 3) = n_q_e.toRotationMatrix();

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * e_covarianceMatrix * J.transpose(), Keys::PosVel<Keys::MotionModelKey>);
    }

    /// @brief Set the Covariance matrix in NED coordinates
    /// @param[in] n_covarianceMatrix 6x6 PosVel Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelCovarianceMatrix_n(const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.rows() == 6, "This function needs a 6x6 matrix as input");
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.cols() == 6, "This function needs a 6x6 matrix as input");

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            n_covarianceMatrix, Keys::PosVel<Keys::MotionModelKey>);

        Eigen::Quaterniond e_q_n = e_Quat_n();
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
        J.block<3, 3>(0, 0) = e_q_n.toRotationMatrix();
        J.block<3, 3>(3, 3) = e_q_n.toRotationMatrix();

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * n_covarianceMatrix * J.transpose(), Keys::PosVel<Keys::MotionModelKey>);
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
