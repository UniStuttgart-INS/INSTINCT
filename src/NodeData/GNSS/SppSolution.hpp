// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppSolution.hpp
/// @brief SPP Algorithm output
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-28

#pragma once

#include <vector>
#include <optional>
#include <algorithm>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"

#include "NodeData/State/PosVel.hpp"

#include "util/Assert.h"
#include "util/Container/KeyedMatrix.hpp"

namespace NAV
{
/// SPP Algorithm output
class SppSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "SppSolution";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    // --------------------------------------------------------- Public Members ------------------------------------------------------------

    /// Amount of satellites used for the calculation
    size_t nSatellites = 0;
    /// Amount of pseudorange measurements used to calculate the position solution
    size_t nMeasPsr = 0;
    /// Amount of doppler measurements used to calculate the velocity solution
    size_t nMeasDopp = 0;
    /// Amount of Parameters estimated in this epoch
    size_t nParam = 0;

    /// Estimated receiver clock parameter
    ReceiverClock recvClk;

    /// Satellite specific data
    struct SatData
    {
        double satElevation = 0.0; ///< Satellite Elevation [rad]
        double satAzimuth = 0.0;   ///< Satellite Azimuth [rad]
    };

    /// Extended data for each satellite
    std::vector<std::pair<SatId, SatData>> satData;

    /// @brief List of events for plotting
    std::vector<std::string> events;

    // ------------------------------------------------------------- Getter ----------------------------------------------------------------

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& e_positionStdev() const { return _e_positionStdev; }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& n_positionStdev() const { return _n_positionStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocityStdev() const { return _n_velocityStdev; }

    /// Returns the Covariance matrix in ECEF frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>>> e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the Covariance matrix in local navigation frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>>> n_CovarianceMatrix() const { return _n_covarianceMatrix; }

    // ------------------------------------------------------------- Setter ----------------------------------------------------------------

    /// @brief Set the Position in ECEF coordinates and its standard deviation
    /// @param[in] e_position New Position in ECEF coordinates [m]
    /// @param[in] e_PositionCovarianceMatrix Standard deviation of Position in ECEF coordinates [m]
    void setPositionAndStdDev_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_PositionCovarianceMatrix)
    {
        setPosition_e(e_position);
        _e_positionStdev = e_PositionCovarianceMatrix.diagonal().cwiseSqrt();
        _n_positionStdev = (n_Quat_e() * e_PositionCovarianceMatrix * e_Quat_n()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    void setVelocityAndStdDev_e(const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityCovarianceMatrix)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityCovarianceMatrix.diagonal().cwiseSqrt();
        _n_velocityStdev = (n_Quat_e() * e_velocityCovarianceMatrix * e_Quat_n()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix Kalman Filter or Least Squares error variance
    /// @attention Position has to be set before calling this
    void setCovarianceMatrix(const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>& e_covarianceMatrix)
    {
        _e_covarianceMatrix = e_covarianceMatrix;
        _n_covarianceMatrix = _e_covarianceMatrix;

        (*_n_covarianceMatrix)(SPP::States::PosVel, all).setZero();
        (*_n_covarianceMatrix)(all, SPP::States::PosVel).setZero();
        Eigen::Vector3d lla_pos = lla_position();
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
        Eigen::Quaterniond e_Quat_n = trafo::e_Quat_n(lla_pos(0), lla_pos(1));
        (*_n_covarianceMatrix)(SPP::States::Pos, SPP::States::Pos) = n_Quat_e * (*_e_covarianceMatrix)(SPP::States::Pos, SPP::States::Pos) * e_Quat_n;
        (*_n_covarianceMatrix)(SPP::States::Vel, SPP::States::Vel) = n_Quat_e * (*_e_covarianceMatrix)(SPP::States::Vel, SPP::States::Vel) * e_Quat_n;
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

    /// Covariance matrix in ECEF coordinates
    std::optional<KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>> _e_covarianceMatrix;

    /// Covariance matrix in local navigation coordinates
    std::optional<KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>> _n_covarianceMatrix;
};

} // namespace NAV
