// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WiFiPositioningSolution.hpp
/// @brief WiFi Positioning Algorithm output
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-03-12

#pragma once

#include "NodeData/State/PosVel.hpp"
#include <Eigen/Dense>

namespace NAV
{
class WiFiPositioningSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "WiFiPositioningSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    // ------------------------------------------------------------- Getter ----------------------------------------------------------------

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& e_positionStdev() const { return _e_positionStdev; }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& n_positionStdev() const { return _n_positionStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocityStdev() const { return _n_velocityStdev; }

    /// Returns the  Covariance matrix in ECEF frame
    [[nodiscard]] const Eigen::MatrixXd& e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the  Covariance matrix in local navigation frame
    [[nodiscard]] const Eigen::MatrixXd& n_CovarianceMatrix() const { return _n_covarianceMatrix; }

    // ------------------------------------------------------------- Setter ----------------------------------------------------------------

    /// @brief Set the Position in ECEF coordinates and its standard deviation
    /// @param[in] e_position New Position in ECEF coordinates [m]
    /// @param[in] e_PositionCovarianceMatrix Standard deviation of Position in ECEF coordinates [m]
    void setPositionAndStdDev_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_PositionCovarianceMatrix)
    {
        setPosition_e(e_position);
        _e_positionStdev = e_PositionCovarianceMatrix.diagonal().cwiseSqrt();
        _n_positionStdev = (n_Quat_e().toRotationMatrix() * e_PositionCovarianceMatrix * n_Quat_e().conjugate().toRotationMatrix()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    void setVelocityAndStdDev_e(const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityCovarianceMatrix)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityCovarianceMatrix.diagonal().cwiseSqrt();
        _n_velocityStdev = (n_Quat_e().toRotationMatrix() * e_velocityCovarianceMatrix * n_Quat_e().conjugate().toRotationMatrix()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Covariance matrix
    /// @param[in] P Covariance matrix
    void setCovarianceMatrix(const Eigen::MatrixXd& P)
    {
        _e_covarianceMatrix = P;
        n_CovarianceMatrix_e();
    }

    /// @brief Transforms the covariance matrix from ECEF frame to local navigation frame
    void n_CovarianceMatrix_e()
    {
        _n_covarianceMatrix = _e_covarianceMatrix;

        Eigen::Vector3d lla_pos = lla_position();
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
        _n_covarianceMatrix.block<3, 3>(0, 0) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix.block<3, 3>(0, 0) * n_Quat_e.conjugate().toRotationMatrix(); // variance of position
        if (_e_covarianceMatrix.rows() >= 4 && _e_covarianceMatrix.cols() >= 4)                                                                                // velocity is also available
        {
            _n_covarianceMatrix.block<3, 3>(3, 3) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix.block<3, 3>(3, 3) * n_Quat_e.toRotationMatrix(); // variance of velocity
        }
    }

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    Eigen::Vector3d _e_positionStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Position in local navigation frame coordinates [m]
    Eigen::Vector3d _n_positionStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Standard deviation of Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Covariance matrix in ECEF coordinates (Position, Velocity)
    Eigen::MatrixXd _e_covarianceMatrix;
    /// Covariance matrix in local navigation coordinates (Position, Velocity)
    Eigen::MatrixXd _n_covarianceMatrix;
};

} // namespace NAV
