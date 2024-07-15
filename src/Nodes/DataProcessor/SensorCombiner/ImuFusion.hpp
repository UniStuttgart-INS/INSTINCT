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

#include "Navigation/INS/SensorCombiner/PinData.hpp"

#include "internal/gui/widgets/TimeEdit.hpp"

#include <deque>
#include <utility>

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

  protected:
    /// Position and rotation information for conversion from platform to body frame
    ImuPos _imuPos;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_COMBINED_SIGNAL = 0; ///< @brief Flow (ImuObs)
    constexpr static size_t OUTPUT_PORT_INDEX_BIASES = 1;          ///< @brief Flow (ImuBiases)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable _nInputPins
    void updateNumberOfInputPins();

    /// @brief Initializes the rotation matrices used for the mounting angles of the sensors
    void initializeMountingAngles();

    /// @brief Receive Function for the signal at the time tₖ
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvSignal(InputPin::NodeDataQueue& queue, size_t pinIdx);

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

    /// @brief Previous observation (for timestamp)
    InsTime _lastFiltObs{};

    /// @brief Calculates the initial measurement noise matrix R
    /// @param[in] R Measurement noise uncertainty matrix for sensor at 'pinIndex'
    /// @param[in] pinIndex Index of pin to identify sensor
    void measurementNoiseMatrix_R(Eigen::MatrixXd& R, size_t pinIndex = 0) const;

    /// @brief Combines the signals
    /// @param[in] imuObs Imu observation
    void combineSignals(const std::shared_ptr<const ImuObs>& imuObs);

    // --------------------------------------- Kalman filter config ------------------------------------------
    /// Number of input pins
    size_t _nInputPins = 2;

    /// @brief Number of states estimated by the IRW-KF (angular rate, angular acceleration, specific force, jerk, all in 3D: 4*3 = 12)
    const uint8_t _numStatesEstIRWKF = 12;

    /// @brief Number of states estimated by the B-spline KF (3 stacked B-splines in 3D for angular rate and specific force: 3*3*2 = 18)
    const uint8_t _numStatesEstBsplineKF = 18;

    /// @brief Number of quadratic B-splines that make up the entire 3D stacked B-spline
    const uint8_t _numBsplines = 9;

    /// @brief Number of states per pin (biases of accel and gyro)
    const uint8_t _numStatesPerPin = 6;

    /// @brief Number of measurements overall
    const uint8_t _numMeasurements = 6;

    /// @brief Number of estimated states (depends on imuFusionType)
    uint8_t _numStatesEst{};

    /// @brief Number of states overall
    uint8_t _numStates = 12;

    /// Kalman Filter representation
    KalmanFilter _kalmanFilter{ _numStates, _numMeasurements };

    // ---------------------------------------- Kalman filter init -------------------------------------------
    /// @brief Highest IMU sample rate in [Hz] (for time step in KF prediction)
    double _imuFrequency{ 100 };

    /// @brief Time until averaging ends and filtering starts in [s]
    double _averageEndTime{ 1 };

    /// @brief Time until averaging ends and filtering starts as 'InsTime'
    InsTime _avgEndTime;

    /// @brief Auto-initialize the Kalman Filter - GUI setting
    bool _autoInitKF = true;

    /// @brief If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
    bool _imuCharacteristicsIdentical = true;

    /// @brief If the multiple IMUs have the same bias, GUI input cells can be reduced considerably
    bool _imuBiasesIdentical = true;

    /// @brief Container that collects all imuObs for averaging for auto-init of the KF
    std::vector<std::shared_ptr<const NAV::ImuObs>> _cumulatedImuObs;

    /// @brief Container that collects all pinIds for averaging for auto-init of the KF
    std::vector<size_t> _cumulatedPinIds;

    /// @brief flag to determine how jerk and angular acceleration states are initialized if '_autoInitKF = true'
    bool _initJerkAngAcc = true;

    /// @brief flag to check whether KF has been auto-initialized
    bool _kfInitialized = false;

    // -------------------------------------------- Sensor info ----------------------------------------------
    /// @brief Stores parameter data for each connected sensor
    std::vector<PinData> _pinData;
    /// @brief Stores IRW-KF specific parameter data
    PinDataIRWKF _pinDataIRWKF;
    /// @brief Stores Bspline-KF specific parameter data
    PinDataBsplineKF _pinDataBsplineKF;

    /// @brief Temporary vector for the initial coefficients for angular rate
    Eigen::Vector3d _initCoeffsAngRateTemp{};
    /// @brief Temporary vector for the initial coefficients for acceleration
    Eigen::Vector3d _initCoeffsAccelTemp{};
    /// @brief Temporary vector for the initial coefficients' initial covariance for the angular rate
    Eigen::Vector3d _initCovarianceCoeffsAngRateTemp{};
    /// @brief Temporary vector for the initial coefficients' initial covariance for the acceleration
    Eigen::Vector3d _initCovarianceCoeffsAccelTemp{};
    /// @brief Temporary vector for the initial coefficients' process noise for the angular rate
    Eigen::Vector3d _procNoiseCoeffsAngRateTemp{};
    /// @brief Temporary vector for the initial coefficients' process noise for the acceleration
    Eigen::Vector3d _procNoiseCoeffsAccelTemp{};

    /// @brief Container for the bias covariances
    std::vector<Eigen::Vector3d> _biasCovariances;
    /// @brief Container for process noise of each state
    std::vector<Eigen::VectorXd> _processNoiseVariances;
    /// @brief Container for measurement noises of each sensor
    std::vector<Eigen::Vector3d> _measurementNoiseVariances;

    /// @brief Rotations of all connected accelerometers - key: pinIndex, value: Rotation matrix of the accelerometer platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_accel;

    /// @brief Rotations of all connected gyros - key: pinIndex, value: Rotation matrix of the gyro platform to body frame
    std::vector<Eigen::Matrix3d> _imuRotations_gyro;

    // ----------------------------------------------- Time --------------------------------------------------
    /// @brief Saves the timestamp of the measurement before in [s]
    InsTime _latestTimestamp{};

    /// @brief Saves the first timestamp in [s]
    InsTime _firstTimestamp{};

    /// @brief Time since startup in [s]
    double _timeSinceStartup{};

    /// @brief Latest knot in [s]
    double _latestKnot{};

    /// @brief Time difference between two quadratic B-splines in the stacked B-spline
    double _splineSpacing = 1.0;

    // ------------------------------------------- Miscellaneous ---------------------------------------------
    /// @brief Check the rank of the Kalman matrices every iteration (computationally expensive)
    bool _checkKalmanMatricesRanks = false;

    /// @brief Check whether the combined solution has an '_imuPos' set
    bool _imuPosSet = false;

    /// Possible KF-types for the IMU fusion
    enum class ImuFusionType
    {
        IRWKF,   ///< IRW Kalman filter
        Bspline, ///< B-spline Kalman filter
        COUNT,   ///< Number of items in the enum
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    friend constexpr const char* to_string(ImuFusionType value);

    /// KF-type for the IMU fusion, selected in the GUI
    ImuFusionType _imuFusionType = ImuFusionType::Bspline;
};

constexpr const char* to_string(NAV::ImuFusion::ImuFusionType value)
{
    switch (value)
    {
    case NAV::ImuFusion::ImuFusionType::IRWKF:
        return "IRW KF";
    case NAV::ImuFusion::ImuFusionType::Bspline:
        return "B-spline KF";
    case NAV::ImuFusion::ImuFusionType::COUNT:
        return "";
    }
    return "";
}

} // namespace NAV
