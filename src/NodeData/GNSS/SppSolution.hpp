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

#include "util/Container/KeyedMatrix.hpp"
#include "Navigation/GNSS/Positioning/SPP/SppKeys.hpp"

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
    /// Satellite system used for the estimation besides Reference time satellite system
    std::vector<SatelliteSystem> usedSatSysExceptRef;

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
    [[nodiscard]] const KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the  Covariance matrix in local navigation frame
    [[nodiscard]] const KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& n_CovarianceMatrix() const { return _n_covarianceMatrix; }

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
    void setPositionClockErrorCovarianceMatrix(const KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& Q)
    {
        _e_positionClockErrorCovarianceMatrix = Q;
    }

    /// @brief Set the Covariance matrix of Least-squares estimation from pseudorange-rate measurements
    /// @param[in] Q lsq variance
    void setVelocityClockDriftCovarianceMatrix(const KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& Q)
    {
        _e_velocityClockDriftCovarianceMatrix = Q;
    }

    /// @brief Set the Covariance matrix of Kalman Filter estimation
    /// @param[in] Q Kalman Filter error variance
    void setCovarianceMatrix(const KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes>& Q)
    {
        _e_covarianceMatrix = Q;
        std::vector<GNSS::Positioning::SPP::States::StateKeyTypes> interSysErrs;
        std::vector<GNSS::Positioning::SPP::States::StateKeyTypes> interSysDrifts;
        for (const auto& key : Q.colKeys())
        {
            if (std::get_if<GNSS::Positioning::SPP::States::InterSysErr>(&key))
            {
                interSysErrs.emplace_back(key);
            }
            if (std::get_if<GNSS::Positioning::SPP::States::InterSysDrift>(&key))
            {
                interSysDrifts.emplace_back(key);
            }
        }

        n_CovarianceMatrix_e(interSysErrs, interSysDrifts);
    }

    /// @brief Set the Covariance matrix of Least-squares estimation from pseudorange and pseudorange-rate measurements
    /// @param[in] interSysErrs Inter-system clock error keys
    /// @param[in] interSysDrifts Inter-system clock drift keys
    void setCovarianceMatrix(const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysErrs, const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysDrifts)
    {
        if (!_e_covarianceMatrix.hasAnyCols(GNSS::Positioning::SPP::States::PosVelRecvClk)) // creates GNSS::Positioning::SPP::States for first use
        {
            _e_covarianceMatrix.addRowsCols(GNSS::Positioning::SPP::States::PosVelRecvClk, GNSS::Positioning::SPP::States::PosVelRecvClk);
            _e_covarianceMatrix.addRowsCols(interSysErrs, interSysErrs);
            _e_covarianceMatrix.addRowsCols(interSysDrifts, interSysDrifts);
        }
        else // check whether interSys keys and GNSS::Positioning::SPP::States in _e_covariance are the same and remove or add accordingly (e. g. due to elevation mask)
        {
            for (const auto& key : _e_covarianceMatrix.colKeys())
            {
                if (std::get_if<GNSS::Positioning::SPP::States::InterSysErr>(&key))
                {
                    if (std::find(interSysErrs.begin(), interSysErrs.end(), key) != interSysErrs.end())
                    {
                        continue;
                    }
                    _e_covarianceMatrix.removeCol(key);
                    _e_covarianceMatrix.removeRow(key);
                }
            }
            for (const auto& key : _e_covarianceMatrix.colKeys())
            {
                if (std::get_if<GNSS::Positioning::SPP::States::InterSysDrift>(&key))
                {
                    if (std::find(interSysDrifts.begin(), interSysDrifts.end(), key) != interSysDrifts.end())
                    {
                        continue;
                    }
                    _e_covarianceMatrix.removeCol(key);
                    _e_covarianceMatrix.removeRow(key);
                }
            }
            for (size_t i = 0; i < interSysErrs.size(); i++)
            {
                if (std::find(_e_covarianceMatrix.colKeys().begin(), _e_covarianceMatrix.colKeys().end(), interSysErrs.at(i)) == _e_covarianceMatrix.colKeys().end())
                {
                    _e_covarianceMatrix.addCol(interSysErrs.at(i));
                    _e_covarianceMatrix.addRow(interSysErrs.at(i));
                    _e_covarianceMatrix.addCol(interSysDrifts.at(i));
                    _e_covarianceMatrix.addRow(interSysDrifts.at(i));
                }
            }
        }

        _e_covarianceMatrix(GNSS::Positioning::SPP::States::PosRecvClkErr, GNSS::Positioning::SPP::States::PosRecvClkErr) = _e_positionClockErrorCovarianceMatrix(GNSS::Positioning::SPP::States::PosRecvClkErr, GNSS::Positioning::SPP::States::PosRecvClkErr);
        _e_covarianceMatrix(interSysErrs, interSysErrs) = _e_positionClockErrorCovarianceMatrix(interSysErrs, interSysErrs);
        _e_covarianceMatrix(GNSS::Positioning::SPP::States::PosRecvClkErr, interSysErrs) = _e_positionClockErrorCovarianceMatrix(GNSS::Positioning::SPP::States::PosRecvClkErr, interSysErrs);
        _e_covarianceMatrix(interSysErrs, GNSS::Positioning::SPP::States::PosRecvClkErr) = _e_positionClockErrorCovarianceMatrix(interSysErrs, GNSS::Positioning::SPP::States::PosRecvClkErr);

        if (_e_velocityClockDriftCovarianceMatrix.cols() > 0)
        {
            _e_covarianceMatrix(GNSS::Positioning::SPP::States::VelRecvClkDrift, GNSS::Positioning::SPP::States::VelRecvClkDrift) = _e_velocityClockDriftCovarianceMatrix(GNSS::Positioning::SPP::States::VelRecvClkDrift, GNSS::Positioning::SPP::States::VelRecvClkDrift);
            _e_covarianceMatrix(interSysDrifts, interSysDrifts) = _e_velocityClockDriftCovarianceMatrix(interSysDrifts, interSysDrifts);
            _e_covarianceMatrix(GNSS::Positioning::SPP::States::VelRecvClkDrift, interSysDrifts) = _e_velocityClockDriftCovarianceMatrix(GNSS::Positioning::SPP::States::VelRecvClkDrift, interSysDrifts);
            _e_covarianceMatrix(interSysDrifts, GNSS::Positioning::SPP::States::VelRecvClkDrift) = _e_velocityClockDriftCovarianceMatrix(interSysDrifts, GNSS::Positioning::SPP::States::VelRecvClkDrift);
        }

        n_CovarianceMatrix_e(interSysErrs, interSysDrifts);
    }

    /// @brief Transforms the covariance matrix from ECEF frame to local navigation frame
    /// @param[in] interSysErrs Inter-system clock error keys
    /// @param[in] interSysDrifts Inter-system clock drift keys
    void n_CovarianceMatrix_e(const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysErrs, const std::vector<GNSS::Positioning::SPP::States::StateKeyTypes>& interSysDrifts)
    {
        _n_covarianceMatrix = _e_covarianceMatrix;

        _n_covarianceMatrix(GNSS::Positioning::SPP::States::PosVel, GNSS::Positioning::SPP::States::PosVel).setZero();
        Eigen::Vector3d lla_pos = lla_position();
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
        _n_covarianceMatrix(GNSS::Positioning::SPP::States::Pos, GNSS::Positioning::SPP::States::Pos) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix(GNSS::Positioning::SPP::States::Pos, GNSS::Positioning::SPP::States::Pos) * n_Quat_e.conjugate().toRotationMatrix(); // variance of position
        _n_covarianceMatrix(GNSS::Positioning::SPP::States::Vel, GNSS::Positioning::SPP::States::Vel) = n_Quat_e.toRotationMatrix() * _e_covarianceMatrix(GNSS::Positioning::SPP::States::Vel, GNSS::Positioning::SPP::States::Vel) * n_Quat_e.toRotationMatrix();             // variance of velocity

        _n_covarianceMatrix(GNSS::Positioning::SPP::States::PosVel, GNSS::Positioning::SPP::States::RecvClk).setZero();
        _n_covarianceMatrix(GNSS::Positioning::SPP::States::RecvClk, GNSS::Positioning::SPP::States::PosVel).setZero();
        _n_covarianceMatrix(GNSS::Positioning::SPP::States::PosVel, interSysErrs).setZero();
        _n_covarianceMatrix(interSysErrs, GNSS::Positioning::SPP::States::PosVel).setZero();
        _n_covarianceMatrix(GNSS::Positioning::SPP::States::PosVel, interSysDrifts).setZero();
        _n_covarianceMatrix(interSysDrifts, GNSS::Positioning::SPP::States::PosVel).setZero();
    }

    /// Extended data structure
    struct SatelliteData
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (code and satellite number)
        explicit SatelliteData(const SatSigId& satSigId) : satSigId(satSigId) {}

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
        double dpsr_ie{ std::nan("") };       ///< Sagnac correction [m]
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
    KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes> _e_positionClockErrorCovarianceMatrix;
    /// Covariance matrix in ECEF coordinates (Velocity and clock drifts from LSE)
    KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes> _e_velocityClockDriftCovarianceMatrix;

    /// Covariance matrix in ECEF coordinates (Position, Velocity, clock parameter (order as in Kalman Filter estimation: position, velocity, receiver clock error, receiver clock drift, inter-system clock error, inter-system clock drift))
    KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes> _e_covarianceMatrix;
    /// Covariance matrix in local navigation coordinates (Position, Velocity, clock parameter (order as in Kalman Filter estimation: position, velocity, receiver clock error, receiver clock drift, inter-system clock error, inter-system clock drift))
    KeyedMatrixXd<GNSS::Positioning::SPP::States::StateKeyTypes, GNSS::Positioning::SPP::States::StateKeyTypes> _n_covarianceMatrix;
};

} // namespace NAV
