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
#include <algorithm>
#include "util/Assert.h"
#include "NodeData/State/PosVel.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"

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

    /// Amount of satellites used for the position calculation
    size_t nSatellitesPosition = 0;
    /// Amount of satellites used for the velocity calculation
    size_t nSatellitesVelocity = 0;

    /// Estimated receiver clock parameter
    ReceiverClock recvClk = { .bias = { std::nan(""), std::nan("") },
                              .drift = { std::nan(""), std::nan("") },
                              .sysTimeDiff = {},
                              .sysDriftDiff = {} };

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

    /// @brief Set the Covariance matrix of Least-squares estimation from pseudorange measurements
    /// @param[in] Q lsq variance
    void setPositionClockErrorCovarianceMatrix(const Eigen::MatrixXd& Q)
    {
        _e_positionClockErrorCovarianceMatrix = Q;
    }

    /// @brief Set the Covariance matrix of Least-squares estimation from pseudorange-rate measurements
    /// @param[in] Q lsq variance
    void setVelocityClockDriftCovarianceMatrix(const Eigen::MatrixXd& Q)
    {
        _e_velocityClockDriftCovarianceMatrix = Q;
    }

    /// @brief Set the Covariance matrix of Kalman Filter estimation
    /// @param[in] Q Kalman Filter error variance
    void setCovarianceMatrix(const Eigen::MatrixXd& Q)
    {
        _e_covarianceMatrix = Q;
        n_CovarianceMatrix_e();
    }

    /// @brief Set the Covariance matrix of Least-squares estimation from pseudorange and pseudorange-rate measurements in the same order as Kalman Filter estimation (position, velocity, receiver clock error, receiver clock drift, inter-system clock error, inter-system clock drift))
    void setCovarianceMatrix()
    {
        Eigen::Index n = 2 * _e_positionClockErrorCovarianceMatrix.cols();
        _e_covarianceMatrix = Eigen::MatrixXd::Zero(n, n);

        _e_covarianceMatrix.block<3, 3>(0, 0) = _e_positionClockErrorCovarianceMatrix.block<3, 3>(0, 0); // variance of position
        _e_covarianceMatrix(6, 6) = _e_positionClockErrorCovarianceMatrix(3, 3);                         // variance of receiver clock error
        _e_covarianceMatrix.block<3, 1>(0, 6) = _e_positionClockErrorCovarianceMatrix.block<3, 1>(0, 3); // covariances between position and receiver clock error
        _e_covarianceMatrix.block<1, 3>(6, 0) = _e_positionClockErrorCovarianceMatrix.block<1, 3>(3, 0); // covariances between position and receiver clock error

        if (_e_velocityClockDriftCovarianceMatrix.cols() > 0)
        {
            _e_covarianceMatrix.block<3, 3>(3, 3) = _e_velocityClockDriftCovarianceMatrix.block<3, 3>(0, 0); // variance of velocity
            _e_covarianceMatrix(7, 7) = _e_velocityClockDriftCovarianceMatrix(3, 3);                         // variance of receiver clock drift
            _e_covarianceMatrix.block<3, 1>(3, 7) = _e_velocityClockDriftCovarianceMatrix.block<3, 1>(0, 3); // covariances between velocity and receiver clock drift
            _e_covarianceMatrix.block<1, 3>(7, 3) = _e_velocityClockDriftCovarianceMatrix.block<1, 3>(3, 0); // covariances between velocity and receiver clock drift}

            auto n_otherSatSys = _e_positionClockErrorCovarianceMatrix.cols() - 4; // number of other satellite systems besides time reference system

            Eigen::Index i_lsq = 0;
            Eigen::Index j_lsq = 0;

            for (Eigen::Index i = 0; i < n_otherSatSys; i++)
            {
                for (Eigen::Index j = 0; j < n_otherSatSys; j++)
                {
                    _e_covarianceMatrix(8 + 2 * i, 8 + 2 * j) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 4 + j_lsq); // variances and covariances of inter-system clock error

                    if (_e_positionClockErrorCovarianceMatrix.cols() > 0)
                    {
                        _e_covarianceMatrix(9 + 2 * i, 9 + 2 * j) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 4 + j_lsq); // variances and covariances of inter-system clock drift
                    }
                    j_lsq += 1;
                }

                // covariances between inter-system clock estimates and position, velocity and receiver clock
                _e_covarianceMatrix(8 + 2 * i, 0) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 0);
                _e_covarianceMatrix(8 + 2 * i, 1) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 1);
                _e_covarianceMatrix(8 + 2 * i, 2) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 2);
                _e_covarianceMatrix(8 + 2 * i, 6) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 3);
                _e_covarianceMatrix(0, 8 + 2 * i) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 0);
                _e_covarianceMatrix(1, 8 + 2 * i) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 1);
                _e_covarianceMatrix(2, 8 + 2 * i) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 2);
                _e_covarianceMatrix(6, 8 + 2 * i) = _e_positionClockErrorCovarianceMatrix(4 + i_lsq, 3);

                if (_e_velocityClockDriftCovarianceMatrix.cols() > 0)
                {
                    _e_covarianceMatrix(9 + 2 * i, 3) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 0);
                    _e_covarianceMatrix(9 + 2 * i, 4) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 1);
                    _e_covarianceMatrix(9 + 2 * i, 5) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 2);
                    _e_covarianceMatrix(9 + 2 * i, 7) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 3);
                    _e_covarianceMatrix(3, 9 + 2 * i) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 0);
                    _e_covarianceMatrix(4, 9 + 2 * i) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 1);
                    _e_covarianceMatrix(5, 9 + 2 * i) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 2);
                    _e_covarianceMatrix(7, 9 + 2 * i) = _e_velocityClockDriftCovarianceMatrix(4 + i_lsq, 3);
                }

                i_lsq += 1;
                j_lsq = 0;
            }
        }
        n_CovarianceMatrix_e();
    }

    /// @brief Transforms the covariance matrix from ECEF frame to local navigation frame
    void n_CovarianceMatrix_e()
    {
        _n_covarianceMatrix = Eigen::MatrixXd::Zero(_e_covarianceMatrix.cols(), _e_covarianceMatrix.cols());

        Eigen::Vector3d lla_pos = lla_position();
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
        _n_covarianceMatrix.block<3, 3>(0, 0) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix.block<3, 3>(0, 0) * n_Quat_e.conjugate().toRotationMatrix(); // variance of position
        _n_covarianceMatrix(6, 6) = _e_covarianceMatrix(6, 6);

        _n_covarianceMatrix.block<3, 3>(3, 3) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix.block<3, 3>(3, 3) * n_Quat_e.toRotationMatrix(); // variance of velocity
        _n_covarianceMatrix(7, 7) = _e_covarianceMatrix(7, 7);

        auto n_otherSatSys = _n_covarianceMatrix.cols() / 2 - 4; // number of other satellite systems besides time reference system

        for (Eigen::Index i = 0; i < n_otherSatSys; i++)
        {
            for (Eigen::Index j = 0; j < n_otherSatSys; j++)
            {
                _n_covarianceMatrix(8 + 2 * i, 8 + 2 * j) = _e_covarianceMatrix(8 + 2 * i, 8 + 2 * j); // variances and covariances of inter-system clock error
                _n_covarianceMatrix(9 + 2 * i, 9 + 2 * j) = _e_covarianceMatrix(8 + 2 * i, 8 + 2 * j); // variances and covariances of inter-system clock drift
            }
        }
    }

    /// Extended data structure
    struct SatelliteData
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (code and satellite number)
        SatelliteData(const SatSigId& satSigId) : satSigId(satSigId) {}

        SatSigId satSigId = { Code::None, 0 }; ///< Code and satellite number

        InsTime transmitTime{};              ///< Time when the signal was transmitted
        Eigen::Vector3d e_satPos;            ///< Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;            ///< Satellite velocity in ECEF frame coordinates [m/s]
        double satClkBias{ 0.0 };            ///< Satellite clock bias [s]
        double satClkDrift{ 0.0 };           ///< Satellite clock drift [s/s]
        double satElevation{ std::nan("") }; ///< Elevation [rad]
        double satAzimuth{ std::nan("") };   ///< Azimuth [rad]
        bool skipped = false;                ///< Bool to check whether the observation was skipped (signal unhealthy)
        bool elevationMaskTriggered = false; ///< Bool to check whether the elevation mask was triggered

        double psrEst{ std::nan("") };        ///< Estimated Pseudorange [m]
        std::optional<double> psrRateEst;     ///< Estimated Pseudorange rate [m/s]
        double geometricDist{ std::nan("") }; ///< Geometric distance [m]
        double dpsr_clkISB{ std::nan("") };   ///< Estimated Inter-system clock bias [m]
        double dpsr_I{ std::nan("") };        ///< Estimated ionosphere propagation error [m]
        double dpsr_T{ std::nan("") };        ///< Estimated troposphere propagation error [m]
        double dpsr_ie{ std::nan("") };       ///< Sagnac correction  [m]
    };

    /// @brief Return the element with the identifier or a newly constructed one if it did not exist
    /// @param[in] satSigId Code and satellite number
    /// @return The element found in the observations or a newly constructed one
    SatelliteData& operator()(const SatSigId& satSigId)
    {
        auto iter = std::find_if(satData.begin(), satData.end(), [&satSigId](const SatelliteData& idData) {
            return idData.satSigId == satSigId;
        });
        if (iter != satData.end())
        {
            return *iter;
        }

        satData.emplace_back(satSigId);
        return satData.back();
    }

    /// @brief Return the element with the identifier
    /// @param[in] satSigId Code and satellite number
    /// @return The element found in the observations
    const SatelliteData& operator()(const SatSigId& satSigId) const
    {
        auto iter = std::find_if(satData.begin(), satData.end(), [&satSigId](const SatelliteData& idData) {
            return idData.satSigId == satSigId;
        });

        INS_ASSERT_USER_ERROR(iter != satData.end(), "You can not insert new elements in a const context.");
        return *iter;
    }

    /// @brief Checks if satellite data exists
    /// @param[in] satSigId Code and satellite number
    /// @return True if the data entry exists
    bool hasSatelliteData(const SatSigId& satSigId) const
    {
        auto iter = std::find_if(satData.begin(), satData.end(), [&satSigId](const SatelliteData& satData) {
            return satData.satSigId == satSigId;
        });
        return iter != satData.end();
    }

    /// Extended data for each satellite frequency and code
    std::vector<SatelliteData> satData;

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    Eigen::Vector3d _e_positionStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Position in local navigation frame coordinates [m]
    Eigen::Vector3d _n_positionStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Standard deviation of Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Covariance matrix in ECEF coordinates (Position and clock errors from LSE)
    Eigen::MatrixXd _e_positionClockErrorCovarianceMatrix;
    /// Covariance matrix in ECEF coordinates (Velocity and clock drifts from LSE)
    Eigen::MatrixXd _e_velocityClockDriftCovarianceMatrix;

    /// Covariance matrix in ECEF coordinates (Position, Velocity, clock parameter (order as in Kalman Filter estimation: position, velocity, receiver clock error, receiver clock drift, inter-system clock error, inter-system clock drift))
    Eigen::MatrixXd _e_covarianceMatrix;
    /// Covariance matrix in local navigation coordinates (Position, Velocity, clock parameter (order as in Kalman Filter estimation: position, velocity, receiver clock error, receiver clock drift, inter-system clock error, inter-system clock drift))
    Eigen::MatrixXd _n_covarianceMatrix;
};

} // namespace NAV
