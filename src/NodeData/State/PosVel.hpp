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
#include <Eigen/src/Core/Matrix.h>

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
            if (auto stDev = e_velocityStdev()) { return stDev->get().x(); }
            break;
        case Pos::GetStaticDescriptorCount() + 8: // Y velocity ECEF StDev [m/s]
            if (auto stDev = e_velocityStdev()) { return stDev->get().z(); }
            break;
        case Pos::GetStaticDescriptorCount() + 9: // Z velocity ECEF StDev [m/s]
            if (auto stDev = e_velocityStdev()) { return stDev->get().z(); }
            break;
        case Pos::GetStaticDescriptorCount() + 10: // XY velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*e_CovarianceMatrix())(States::VelX, States::VelY); }
            break;
        case Pos::GetStaticDescriptorCount() + 11: // XZ velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*e_CovarianceMatrix())(States::VelX, States::VelZ); }
            break;
        case Pos::GetStaticDescriptorCount() + 12: // YZ velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*e_CovarianceMatrix())(States::VelY, States::VelZ); }
            break;
        case Pos::GetStaticDescriptorCount() + 13: // North velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->get().x(); }
            break;
        case Pos::GetStaticDescriptorCount() + 14: // East velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->get().y(); }
            break;
        case Pos::GetStaticDescriptorCount() + 15: // Down velocity StDev [m/s]
            if (auto stDev = n_velocityStdev()) { return stDev->get().z(); }
            break;
        case Pos::GetStaticDescriptorCount() + 16: // NE velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*n_CovarianceMatrix())(States::VelX, States::VelY); }
            break;
        case Pos::GetStaticDescriptorCount() + 17: // ND velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*n_CovarianceMatrix())(States::VelX, States::VelZ); }
            break;
        case Pos::GetStaticDescriptorCount() + 18: // ED velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(States::Vel)) { return (*n_CovarianceMatrix())(States::VelY, States::VelZ); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief States
    struct States
    {
        /// @brief Constructor
        States() = delete;

        /// @brief State Keys
        enum StateKeys : uint8_t
        {
            PosX,         ///< Position ECEF_X [m]
            PosY,         ///< Position ECEF_Y [m]
            PosZ,         ///< Position ECEF_Z [m]
            VelX,         ///< Velocity ECEF_X [m/s]
            VelY,         ///< Velocity ECEF_Y [m/s]
            VelZ,         ///< Velocity ECEF_Z [m/s]
            States_COUNT, ///< Count
        };
        /// @brief All position keys
        inline static const std::vector<StateKeys> Pos = { PosX, PosY, PosZ };
        /// @brief All velocity keys
        inline static const std::vector<StateKeys> Vel = { States::VelX, States::VelY, States::VelZ };
        /// @brief Vector with all position and velocity state keys
        inline static const std::vector<StateKeys> PosVel = { States::PosX, States::PosY, States::PosZ,
                                                              States::VelX, States::VelY, States::VelZ };
    };

    /// Returns the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocity() const { return _e_velocity; }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocity() const { return _n_velocity; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] std::optional<std::reference_wrapper<const Eigen::Vector3d>> e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] std::optional<std::reference_wrapper<const Eigen::Vector3d>> n_velocityStdev() const { return _n_velocityStdev; }

    /// Returns the Covariance matrix in ECEF frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<States::StateKeys, States::StateKeys>>> e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the Covariance matrix in local navigation frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<States::StateKeys, States::StateKeys>>> n_CovarianceMatrix() const { return _n_covarianceMatrix; }

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

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    template<typename Derived, typename Derived2>
    void setVelocityAndStdDev_e(const Eigen::MatrixBase<Derived>& e_velocity, const Eigen::MatrixBase<Derived2>& e_velocityCovarianceMatrix)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityCovarianceMatrix.diagonal().cwiseSqrt();
        _n_velocityStdev = (n_Quat_e() * e_velocityCovarianceMatrix * e_Quat_n()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Velocity in NED coordinates and its standard deviation
    /// @param[in] n_velocity New Velocity in NED coordinates [m/s]
    /// @param[in] n_velocityCovarianceMatrix Covariance matrix of Velocity in navigation coordinates [m/s]
    template<typename Derived, typename Derived2>
    void setVelocityAndStdDev_n(const Eigen::MatrixBase<Derived>& n_velocity, const Eigen::MatrixBase<Derived2>& n_velocityCovarianceMatrix)
    {
        setVelocity_n(n_velocity);
        _n_velocityStdev = n_velocityCovarianceMatrix.diagonal().cwiseSqrt();
        _e_velocityStdev = (e_Quat_n() * n_velocityCovarianceMatrix * n_Quat_e()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix 6x6 PosVel Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelCovarianceMatrix_e(const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.rows() == 6, "This function needs a 6x6 matrix as input");
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.cols() == 6, "This function needs a 6x6 matrix as input");

        _e_covarianceMatrix = KeyedMatrixXd<States::StateKeys,
                                            States::StateKeys>(e_covarianceMatrix,
                                                               States::PosVel);
        _n_covarianceMatrix = _e_covarianceMatrix;

        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(latitude(), longitude());
        Eigen::Quaterniond e_Quat_n = trafo::e_Quat_n(latitude(), longitude());

        (*_n_covarianceMatrix)(all, all).setZero();
        (*_n_covarianceMatrix)(States::Pos, States::Pos) = n_Quat_e * (*_e_covarianceMatrix)(States::Pos, States::Pos) * e_Quat_n;
        (*_n_covarianceMatrix)(States::Vel, States::Vel) = n_Quat_e * (*_e_covarianceMatrix)(States::Vel, States::Vel) * e_Quat_n;
    }

    /// @brief Set the Covariance matrix in NED coordinates
    /// @param[in] n_covarianceMatrix 6x6 PosVel Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosVelCovarianceMatrix_n(const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.rows() == 6, "This function needs a 6x6 matrix as input");
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.cols() == 6, "This function needs a 6x6 matrix as input");

        _n_covarianceMatrix = KeyedMatrixXd<States::StateKeys,
                                            States::StateKeys>(n_covarianceMatrix,
                                                               States::PosVel);
        _e_covarianceMatrix = _n_covarianceMatrix;

        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(latitude(), longitude());
        Eigen::Quaterniond e_Quat_n = trafo::e_Quat_n(latitude(), longitude());

        (*_e_covarianceMatrix)(all, all).setZero();
        (*_e_covarianceMatrix)(States::Pos, States::Pos) = e_Quat_n * (*_n_covarianceMatrix)(States::Pos, States::Pos) * n_Quat_e;
        (*_e_covarianceMatrix)(States::Vel, States::Vel) = e_Quat_n * (*_n_covarianceMatrix)(States::Vel, States::Vel) * n_Quat_e;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocity{ std::nan(""), std::nan(""), std::nan("") };
    /// Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocity{ std::nan(""), std::nan(""), std::nan("") };

    /// Standard deviation of Velocity in earth coordinates [m/s]
    std::optional<Eigen::Vector3d> _e_velocityStdev;
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    std::optional<Eigen::Vector3d> _n_velocityStdev;

    /// Covariance matrix in ECEF coordinates
    std::optional<KeyedMatrixXd<States::StateKeys, States::StateKeys>> _e_covarianceMatrix;

    /// Covariance matrix in local navigation coordinates
    std::optional<KeyedMatrixXd<States::StateKeys, States::StateKeys>> _n_covarianceMatrix;
};

} // namespace NAV
