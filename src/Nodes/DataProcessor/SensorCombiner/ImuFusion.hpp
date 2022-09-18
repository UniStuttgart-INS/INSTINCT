// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuFusion.hpp
/// @brief Combines signals of sensors that provide the same signal-type to one signal
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-03-24

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "Navigation/Math/KalmanFilter.hpp"

namespace NAV
{
/// @brief Combines signals of sensors that provide the same signal-type to one signal
class ImuFusion : public Imu
{
  public:
    /// @brief Default constructor
    ImuFusion();
    /// @brief Destructor
    ~ImuFusion() override;
    /// @brief Copy constructor
    ImuFusion(const ImuFusion&) = delete;
    /// @brief Move constructor
    ImuFusion(ImuFusion&&) = delete;
    /// @brief Copy assignment operator
    ImuFusion& operator=(const ImuFusion&) = delete;
    /// @brief Move assignment operator
    ImuFusion& operator=(ImuFusion&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Information about a sensor which is connected to a certain pin
    struct PinData
    {
        // ---------------------------------------- Variance Units -------------------------------------------
        /// Possible Units for the variance for the process noise of the angular rate (standard deviation σ or Variance σ²)
        enum class AngRateVarianceUnit
        {
            rad2_s2, ///< Variance [rad²/s², rad²/s², rad²/s²]
            rad_s,   ///< Standard deviation [rad/s, rad/s, rad/s]
            deg2_s2, ///< Variance [deg²/s², deg²/s², deg²/s²]
            deg_s,   ///< Standard deviation [deg/s, deg/s, deg/s]
        };

        /// Possible Units for the variance for the process noise of the acceleration (standard deviation σ or Variance σ²)
        enum class AccelerationVarianceUnit
        {
            m2_s4, ///< Variance [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
            m_s2,  ///< Standard deviation [m/s², m/s², m/s²]
        };

        /// Possible Units for the variance for the process noise of the angular acceleration (standard deviation σ or Variance σ²)
        enum class AngularAccVarianceUnit
        {
            rad2_s4, ///< Variance [(rad^2)/(s^4), (rad^2)/(s^4), (rad^2)/(s^4)]
            rad_s2,  ///< Standard deviation [rad/s², rad/s², rad/s²]
            deg2_s4, ///< Variance [(deg^2)/(s^4), (deg^2)/(s^4), (deg^2)/(s^4)]
            deg_s2,  ///< Standard deviation [deg/s², deg/s², deg/s²]
        };

        /// Possible Units for the variance for the process noise of the jerk (standard deviation σ or Variance σ²)
        enum class JerkVarianceUnit
        {
            m2_s6, ///< Variance [(m^2)/(s^6), (m^2)/(s^6), (m^2)/(s^6)]
            m_s3,  ///< Standard deviation [m/s³, m/s³, m/s³]
        };

        // --------------------------------- State unit and initialization -----------------------------------
        /// GUI selection of the initial covariance diagonal values for angular rate (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceAngularRate{ 1, 1, 1 };
        /// GUI selection of the initial covariance diagonal values for angular acceleration (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceAngularAcc{ 0.1, 0.1, 0.1 };
        /// GUI selection of the initial covariance diagonal values for acceleration (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceAcceleration{ 0.1, 0.1, 0.1 };
        /// GUI selection of the initial covariance diagonal values for jerk (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceJerk{ 0.1, 0.1, 0.1 };
        /// GUI selection of the initial covariance diagonal values for angular rate biases (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceBiasAngRate{ 1, 1, 1 };
        /// GUI selection of the initial covariance diagonal values for acceleration biases (standard deviation σ or Variance σ²)
        Eigen::Vector3d initCovarianceBiasAcc{ 0.1, 0.1, 0.1 };
        /// GUI selection of the angular acceleration process noise diagonal values
        Eigen::Vector3d varAngularAccNoise{ 0.1, 0.1, 0.1 };
        /// GUI selection of the jerk process noise diagonal values
        Eigen::Vector3d varJerkNoise{ 0.1, 0.1, 0.1 };
        /// GUI selection of the process noise of the angular rate diagonal values (standard deviation σ or Variance σ²)
        Eigen::Vector3d varBiasAngRateNoise = { 1, 1, 1 };
        /// GUI selection of the process noise of the acceleration diagonal values (standard deviation σ or Variance σ²)
        Eigen::Vector3d varBiasAccelerationNoise{ 0.1, 0.1, 0.1 };
        /// Gui selection of the angular rate measurement uncertainty diagonal values
        Eigen::Vector3d measurementUncertaintyAngularRate{ 1, 1, 1 };
        /// Gui selection of the angular acceleration measurement uncertainty diagonal values
        Eigen::Vector3d measurementUncertaintyAcceleration{ 0.1, 0.1, 0.1 };

        /// Gui selection for the Unit of the initial covariance for the angular rate
        AngRateVarianceUnit initCovarianceAngularRateUnit = AngRateVarianceUnit::deg_s;
        /// Gui selection for the Unit of the initial covariance for the angular acceleration
        AngularAccVarianceUnit initCovarianceAngularAccUnit = AngularAccVarianceUnit::deg_s2;
        /// Gui selection for the Unit of the initial covariance for the acceleration
        AccelerationVarianceUnit initCovarianceAccelerationUnit = AccelerationVarianceUnit::m_s2;
        /// Gui selection for the Unit of the initial covariance for the jerk
        JerkVarianceUnit initCovarianceJerkUnit = JerkVarianceUnit::m_s3;
        /// Gui selection for the Unit of the initial covariance for the angular rate biases
        AngRateVarianceUnit initCovarianceBiasAngRateUnit = AngRateVarianceUnit::deg_s;
        /// Gui selection for the Unit of the initial covariance for the acceleration biases
        AccelerationVarianceUnit initCovarianceBiasAccUnit = AccelerationVarianceUnit::m_s2;
        /// Gui selection for the Unit of the angular acceleration process noise
        AngularAccVarianceUnit varAngularAccNoiseUnit = AngularAccVarianceUnit::deg_s2;
        /// Gui selection for the Unit of the jerk process noise
        JerkVarianceUnit varJerkNoiseUnit = JerkVarianceUnit::m_s3;
        /// Gui selection for the Unit of the process noise of the angular rate
        AngRateVarianceUnit varBiasAngRateNoiseUnit = AngRateVarianceUnit::deg_s;
        /// Gui selection for the Unit of the process noise of the acceleration
        AccelerationVarianceUnit varBiasAccelerationNoiseUnit = AccelerationVarianceUnit::m_s2;
        /// Gui selection for the unit of the angular rate's measurement uncertainty
        AngRateVarianceUnit measurementUncertaintyAngularRateUnit = AngRateVarianceUnit::deg_s;
        /// Gui selection for the unit of the acceleration's measurement uncertainty
        AccelerationVarianceUnit measurementUncertaintyAccelerationUnit = AccelerationVarianceUnit::m_s2;
    };

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_COMBINED_SIGNAL = 0; ///< @brief Flow (ImuObs)
    constexpr static size_t OUTPUT_PORT_INDEX_BIASES = 1;          ///< @brief Flow (ImuBiases)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable _nInputPins
    void updateNumberOfInputPins();

    /// @brief Initializes the Kalman Filter
    void initializeKalmanFilter();

    /// @brief Initializes the rotation matrices used for the mounting angles of the sensors
    void initializeMountingAngles();

    /// @brief Receive Function for the signal at the time tₖ
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvSignal(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Calculates the state-transition-matrix 𝚽
    /// @param[in] dt Time difference between two successive measurements
    /// @return State-transition-matrix 𝚽
    [[nodiscard]] Eigen::MatrixXd initialStateTransitionMatrix_Phi(double dt) const;

    /// @brief Calculates the state-transition-matrix 𝚽
    /// @param[in] Phi State transition matrix from previous iteration. Returns the matrix for the current iteration.
    /// @param[in] dt Time difference between two successive measurements
    void static stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, double dt);

    /// @brief Calculates the process noise matrix Q
    /// @param[in] Q Process noise matrix of the previous time step
    /// @param[in] dt Time difference between two successive measurements
    void processNoiseMatrix_Q(Eigen::MatrixXd& Q, double dt) const;

    /// @brief Calculates the design matrix H
    /// @param[in] DCM_accel Rotation matrix of mounting angles of an accelerometer w.r.t. a common reference
    /// @param[in] DCM_gyro Rotation matrix of mounting angles of a gyroscope w.r.t. a common reference
    /// @param[in] pinIndex Index of pin to identify sensor
    /// @return Design matrix H
    [[nodiscard]] Eigen::MatrixXd designMatrix_H(const Eigen::Matrix3d& DCM_accel, const Eigen::Matrix3d& DCM_gyro, size_t pinIndex) const;

    /// @brief Calculates the adaptive measurement noise matrix R
    /// @param[in] alpha Forgetting factor (i.e. weight on previous estimates), 0 < alpha < 1
    /// @param[in] R Measurement noise covariance matrix at the previous epoch
    /// @param[in] e Vector of residuals
    /// @param[in] H Design matrix
    /// @param[in] P Error covariance matrix
    /// @return Measurement noise matrix R
    /// @note See https://arxiv.org/pdf/1702.00884.pdf
    [[nodiscard]] static Eigen::MatrixXd measurementNoiseMatrix_R_adaptive(double alpha,
                                                                           const Eigen::MatrixXd& R,
                                                                           const Eigen::VectorXd& e,
                                                                           const Eigen::MatrixXd& H,
                                                                           const Eigen::MatrixXd& P);

    /// @brief Calculates the initial measurement noise matrix R
    /// @param[in] R Measurement noise uncertainty matrix for sensor at 'pinIndex'
    /// @param[in] pinIndex Index of pin to identify sensor
    void measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t pinIndex) const;

    /// @brief Initial error covariance matrix P_0
    /// @param[in] varAngRate Initial variance (3D) of the Angular Rate state in [rad²/s²]
    /// @param[in] varAngAcc Initial variance (3D) of the Angular Acceleration state in [(rad^2)/(s^4)]
    /// @param[in] varAcc Initial variance (3D) of the Acceleration state in [(m^2)/(s^4)]
    /// @param[in] varJerk Initial variance (3D) of the Jerk state in [(m^2)/(s^6)]
    /// @return The (_numStates) x (_numStates) matrix of initial state variances
    [[nodiscard]] Eigen::MatrixXd initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& varAngRate,
                                                                  const Eigen::Vector3d& varAngAcc,
                                                                  const Eigen::Vector3d& varAcc,
                                                                  const Eigen::Vector3d& varJerk) const;

    /// @brief Combines the signals
    /// @param[in] imuObs Imu observation
    void combineSignals(const std::shared_ptr<const ImuObs>& imuObs);

    /// Number of input pins
    size_t _nInputPins = 2;

    /// @brief Flag to check whether the design matrix H has been initialized
    bool _designMatrixInitialized = false;

    /// @brief Number of estimated states (accel and gyro)
    uint8_t _numStatesEst = 12;

    /// @brief Number of states per pin (biases of accel and gyro)
    uint8_t _numStatesPerPin = 6;

    /// @brief Number of states overall
    uint8_t _numStates = 12;

    /// @brief Number of measurements overall
    uint8_t _numMeasurements = 6;

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// @brief Rotations of all connected accelerometers - key: pinIndex, value: Rotation matrix of the accelerometer platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_accel;

    /// @brief Rotations of all connected gyros - key: pinIndex, value: Rotation matrix of the gyro platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_gyro;

    /// Kalman Filter representation
    KalmanFilter _kalmanFilter{ _numStates, _numMeasurements };

    /// @brief Highest IMU sample rate (for time step in KF prediction)
    double _imuFrequency{ 100 };

    /// @brief Saves the timestamp of the measurement before in [s]
    InsTime _latestTimestamp{};

    /// @brief Container for process noise of each state
    std::vector<Eigen::Vector3d> _biasCovariances;

    /// @brief Container for process noise of each state
    std::vector<Eigen::Vector3d> _processNoiseVariances;

    /// @brief Container for measurement noises of each sensor
    std::vector<Eigen::Vector3d> _measurementNoiseVariances;

    /// @brief Check the rank of the Kalman matrices every iteration (computationally expensive)
    bool _checkKalmanMatricesRanks = false;

    /// @brief Check whether the combined solution has an '_imuPos' set
    bool _imuPosSet = false;
};

} // namespace NAV
