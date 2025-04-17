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

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

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
        desc.emplace_back("Roll StdDev [deg]");
        desc.emplace_back("Pitch StdDev [deg]");
        desc.emplace_back("Yaw StdDev [deg]");
        desc.emplace_back("n_Quat_b x");
        desc.emplace_back("n_Quat_b y");
        desc.emplace_back("n_Quat_b z");
        desc.emplace_back("n_Quat_b w");
        desc.emplace_back("n_Quat_b x StdDev");
        desc.emplace_back("n_Quat_b y StdDev");
        desc.emplace_back("n_Quat_b z StdDev");
        desc.emplace_back("n_Quat_b w StdDev");
        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVel::GetStaticDescriptorCount() + 14; }

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
            if (_n_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().x()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 1: // Pitch [deg]
            if (_n_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().y()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 2: // Yaw [deg]
            if (_n_Quat_b.norm() != 0) { return rad2deg(rollPitchYaw().z()); }
            break;
        case PosVel::GetStaticDescriptorCount() + 3: // Roll StdDev [deg]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Att<Keys::MotionModelKey>))
            {
                auto covRPY = trafo::covQuat2RPY((*_n_covarianceMatrix)(Keys::Att<Keys::MotionModelKey>, Keys::Att<Keys::MotionModelKey>), n_Quat_b());
                return rad2deg(std::sqrt(covRPY(0, 0)));
            }
            break;
        case PosVel::GetStaticDescriptorCount() + 4: // Pitch StdDev [deg]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Att<Keys::MotionModelKey>))
            {
                auto covRPY = trafo::covQuat2RPY((*_n_covarianceMatrix)(Keys::Att<Keys::MotionModelKey>, Keys::Att<Keys::MotionModelKey>), n_Quat_b());
                return rad2deg(std::sqrt(covRPY(1, 1)));
            }
            break;
        case PosVel::GetStaticDescriptorCount() + 5: // Yaw StdDev [deg]
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRows(Keys::Att<Keys::MotionModelKey>))
            {
                auto covRPY = trafo::covQuat2RPY((*_n_covarianceMatrix)(Keys::Att<Keys::MotionModelKey>, Keys::Att<Keys::MotionModelKey>), n_Quat_b());
                return rad2deg(std::sqrt(covRPY(2, 2)));
            }
            break;
        case PosVel::GetStaticDescriptorCount() + 6: // n_Quat_b x
            if (_n_Quat_b.norm() != 0) { return _n_Quat_b.x(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 7: // n_Quat_b y
            if (_n_Quat_b.norm() != 0) { return _n_Quat_b.y(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 8: // n_Quat_b z
            if (_n_Quat_b.norm() != 0) { return _n_Quat_b.z(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 9: // n_Quat_b w
            if (_n_Quat_b.norm() != 0) { return _n_Quat_b.w(); }
            break;
        case PosVel::GetStaticDescriptorCount() + 10: // n_Quat_b x StdDev
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRow(Keys::AttQ1)) { return std::sqrt((*_n_covarianceMatrix)(Keys::AttQ1, Keys::AttQ1)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 11: // n_Quat_b y StdDev
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRow(Keys::AttQ2)) { return std::sqrt((*_n_covarianceMatrix)(Keys::AttQ2, Keys::AttQ2)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 12: // n_Quat_b z StdDev
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRow(Keys::AttQ3)) { return std::sqrt((*_n_covarianceMatrix)(Keys::AttQ3, Keys::AttQ3)); }
            break;
        case PosVel::GetStaticDescriptorCount() + 13: // n_Quat_b w StdDev
            if (_n_covarianceMatrix && _n_covarianceMatrix->hasRow(Keys::AttQ0)) { return std::sqrt((*_n_covarianceMatrix)(Keys::AttQ0, Keys::AttQ0)); }
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
        if (idx < PosVel::GetStaticDescriptorCount()) { return PosVel::setValueAt(idx, value); }
        // switch (idx)
        // {
        // case PosVel::GetStaticDescriptorCount() + 0: // Roll [deg]
        // case PosVel::GetStaticDescriptorCount() + 1: // Pitch [deg]
        // case PosVel::GetStaticDescriptorCount() + 2: // Yaw [deg]
        // case PosVel::GetStaticDescriptorCount() + 3: // Quaternion::w
        // case PosVel::GetStaticDescriptorCount() + 4: // Quaternion::x
        // case PosVel::GetStaticDescriptorCount() + 5: // Quaternion::y
        // case PosVel::GetStaticDescriptorCount() + 6: // Quaternion::z
        // default:
        // }

        return false;
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

    /// Returns the standard deviation of the Quaternion body to earth frame (x, y, z, w)
    [[nodiscard]] std::optional<Eigen::Vector4d> e_QuatStdev() const
    {
        if (_e_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Att<Keys::MotionModelKey>))
        {
            return (*_e_covarianceMatrix)(Keys::Att<Keys::MotionModelKey>, Keys::Att<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
    }

    /// Returns the standard deviation of the Quaternion body to navigation frame (roll, pitch, yaw) (x, y, z, w)
    [[nodiscard]] std::optional<Eigen::Vector4d> n_QuatStdev() const
    {
        if (_n_covarianceMatrix && _e_covarianceMatrix->hasRows(Keys::Att<Keys::MotionModelKey>))
        {
            return (*_n_covarianceMatrix)(Keys::Att<Keys::MotionModelKey>, Keys::Att<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
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

    /// @brief Set the position, velocity and attitude
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    template<typename DerivedP, typename DerivedV, typename DerivedA>
    void setPosVelAtt_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedV>& e_velocity, const Eigen::QuaternionBase<DerivedA>& e_Quat_b)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
        setAttitude_e_Quat_b(e_Quat_b);
    }

    /// @brief Set the position, velocity and attitude
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    template<typename DerivedP, typename DerivedV, typename DerivedA>
    void setPosVelAtt_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedV>& n_velocity, const Eigen::QuaternionBase<DerivedA>& n_Quat_b)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
        setAttitude_n_Quat_b(n_Quat_b);
    }

    /// @brief Set the position, velocity, attitude and the covariance matrix
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_velocity The new velocity in the earth frame
    /// @param[in] e_Quat_b Quaternion from body to earth frame
    /// @param[in] e_covarianceMatrix 10x10 PosVelAtt Error variance
    template<typename DerivedP, typename DerivedV, typename DerivedA, typename Derived>
    void setPosVelAttAndCov_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<DerivedV>& e_velocity,
                              const Eigen::QuaternionBase<DerivedA>& e_Quat_b, const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        setPosition_e(e_position);
        setVelocity_e(e_velocity);
        setAttitude_e_Quat_b(e_Quat_b);
        setPosVelAttCovarianceMatrix_e(e_covarianceMatrix);
    }

    /// @brief Set the position, velocity, attitude and the covariance matrix
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_velocity The new velocity in the NED frame [m/s, m/s, m/s]
    /// @param[in] n_Quat_b Quaternion from body to navigation frame
    /// @param[in] n_covarianceMatrix 10x10 PosVelAtt Error variance
    template<typename DerivedP, typename DerivedV, typename DerivedA, typename Derived>
    void setPosVelAttAndCov_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<DerivedV>& n_velocity,
                              const Eigen::QuaternionBase<DerivedA>& n_Quat_b, const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        setPosition_lla(lla_position);
        setVelocity_n(n_velocity);
        setAttitude_n_Quat_b(n_Quat_b);
        setPosVelAttCovarianceMatrix_n(n_covarianceMatrix);
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix 10x10 PosVelAtt Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelAttCovarianceMatrix_e(const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.rows() == 10, "This function needs a 10x10 matrix as input");
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.cols() == 10, "This function needs a 10x10 matrix as input");

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            e_covarianceMatrix, Keys::PosVelAtt<Keys::MotionModelKey>);

        Eigen::Quaterniond n_q_e = n_Quat_e();
        Eigen::Matrix<double, 10, 10> J = Eigen::Matrix<double, 10, 10>::Zero();
        J.block<3, 3>(0, 0) = n_q_e.toRotationMatrix();
        J.block<3, 3>(3, 3) = n_q_e.toRotationMatrix();
        J.block<4, 4>(6, 6) = trafo::covQuat2QuatJacobian(n_q_e);

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * e_covarianceMatrix * J.transpose(), Keys::PosVelAtt<Keys::MotionModelKey>);
    }

    /// @brief Set the Covariance matrix in NED coordinates
    /// @param[in] n_covarianceMatrix 10x10 PosVelAtt Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelAttCovarianceMatrix_n(const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.rows() == 10, "This function needs a 10x10 matrix as input");
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.cols() == 10, "This function needs a 10x10 matrix as input");

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            n_covarianceMatrix, Keys::PosVelAtt<Keys::MotionModelKey>);

        Eigen::Quaterniond e_q_n = e_Quat_n();
        Eigen::Matrix<double, 10, 10> J = Eigen::Matrix<double, 10, 10>::Zero();
        J.block<3, 3>(0, 0) = e_q_n.toRotationMatrix();
        J.block<3, 3>(3, 3) = e_q_n.toRotationMatrix();
        J.block<4, 4>(6, 6) = trafo::covQuat2QuatJacobian(e_q_n);

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * n_covarianceMatrix * J.transpose(), Keys::PosVelAtt<Keys::MotionModelKey>);
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
