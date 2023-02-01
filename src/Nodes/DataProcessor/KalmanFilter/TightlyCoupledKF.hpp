// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TightlyCoupledKF.hpp
/// @brief Kalman Filter class for the tightly coupled INS/GNSS integration
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-18

#pragma once

#include "internal/Node/Node.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

#include "Navigation/Math/KalmanFilter.hpp"

namespace NAV
{
/// @brief Tightly-coupled Kalman Filter for INS/GNSS integration
class TightlyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    TightlyCoupledKF();
    /// @brief Destructor
    ~TightlyCoupledKF() override;
    /// @brief Copy constructor
    TightlyCoupledKF(const TightlyCoupledKF&) = delete;
    /// @brief Move constructor
    TightlyCoupledKF(TightlyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    TightlyCoupledKF& operator=(const TightlyCoupledKF&) = delete;
    /// @brief Move assignment operator
    TightlyCoupledKF& operator=(TightlyCoupledKF&&) = delete;
    /// @brief String representation of the class type
    [[nodiscard]] static std::string typeStatic();
    /// @brief String representation of the class type
    [[nodiscard]] std::string type() const override;
    /// @brief String representation of the class category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t INPUT_PORT_INDEX_GNSS = 1;   ///< @brief Flow (PosVel)
    constexpr static size_t OUTPUT_PORT_INDEX_ERROR = 0; ///< @brief Flow (TcKfInsGnssErrors)
    constexpr static size_t OUTPUT_PORT_INDEX_SYNC = 1;  ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the inertial navigation solution
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvInertialNavigationSolution(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Function for the Gnss observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Predicts the state from the InertialNavSol
    /// @param[in] inertialNavSol Inertial navigation solution triggering the prediction
    /// @param[in] tau_i Time since the last prediction in [s]
    void tightlyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol, double tau_i);

    /// @brief Updates the predicted state from the InertialNavSol with the GNSS observation
    /// @param[in] gnssObservation Gnss observation triggering the update
    void tightlyCoupledUpdate(const std::shared_ptr<const GnssObs>& gnssObservation);

    /// Latest observation from the Inertial Integrator (Position, Velocity, Attitude and IMU measurements)
    std::shared_ptr<const InertialNavSol> _latestInertialNavSol = nullptr;

    /// Time when the last prediction was triggered
    InsTime _lastPredictTime;

    /// Time when the last GNSS message came and a prediction was requested
    InsTime _lastPredictRequestedTime;

    // ###########################################################################################################
    //                                               GUI Settings
    // ###########################################################################################################

    // ###########################################################################################################
    //                                                Prediction
    // ###########################################################################################################

    // ------------------------------------------- System matrix 𝐅 ----------------------------------------------

    /// @brief Calculates the system matrix 𝐅 for the local navigation frame
    /// @param[in] n_Quat_b Attitude of the body with respect to n-system
    /// @param[in] b_specForce_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coordinates
    /// @param[in] n_omega_in Angular rate of navigation system with respect to the inertial system [rad / s], resolved in navigation coordinates.
    /// @param[in] n_velocity Velocity in n-system in [m / s]
    /// @param[in] lla_position Position as Lat Lon Alt in [rad rad m]
    /// @param[in] R_N Meridian radius of curvature in [m]
    /// @param[in] R_E Prime vertical radius of curvature (East/West) [m]
    /// @param[in] g_0 Magnitude of the gravity vector in [m/s^2] (see \cite Groves2013 Groves, ch. 2.4.7, eq. 2.135, p. 70)
    /// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.63)
    [[nodiscard]] Eigen::Matrix<double, 15, 15> n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
                                                                 const Eigen::Vector3d& b_specForce_ib,
                                                                 const Eigen::Vector3d& n_omega_in,
                                                                 const Eigen::Vector3d& n_velocity,
                                                                 const Eigen::Vector3d& lla_position,
                                                                 double R_N,
                                                                 double R_E,
                                                                 double g_0,
                                                                 double r_eS_e,
                                                                 const Eigen::Vector3d& tau_bad,
                                                                 const Eigen::Vector3d& tau_bgd) const;

    /// @brief Calculates the system matrix 𝐅 for the ECEF frame
    /// @param[in] e_Quat_b Attitude of the body with respect to e-system
    /// @param[in] b_specForce_ib Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coordinates
    /// @param[in] e_position Position in ECEF coordinates in [m]
    /// @param[in] e_gravitation Gravitational acceleration in [m/s^2]
    /// @param[in] r_eS_e Geocentric radius. The distance of a point on the Earth's surface from the center of the Earth in [m]
    /// @param[in] e_omega_ie Angular velocity of Earth with respect to inertial system, represented in e-sys in [rad/s]
    /// @param[in] tau_bad Correleation length for the accelerometer in [s]
    /// @param[in] tau_bgd Correleation length for the gyroscope in [s]
    /// @note See Groves (2013) chapter 14.2.3, equation (14.48)
    [[nodiscard]] Eigen::Matrix<double, 15, 15> e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
                                                                 const Eigen::Vector3d& b_specForce_ib,
                                                                 const Eigen::Vector3d& e_position,
                                                                 const Eigen::Vector3d& e_gravitation,
                                                                 double r_eS_e,
                                                                 const Eigen::Vector3d& e_omega_ie,
                                                                 const Eigen::Vector3d& tau_bad,
                                                                 const Eigen::Vector3d& tau_bgd) const;
    // ----------------------------- Noise input matrix 𝐆 & Noise scale matrix 𝐖 -------------------------------
    // ----------------------------------- System noise covariance matrix 𝐐 -------------------------------------

    // --------------------------------------- Error covariance matrix P -----------------------------------------

    // ###########################################################################################################
    //                                                  Update
    // ###########################################################################################################
};

} // namespace NAV