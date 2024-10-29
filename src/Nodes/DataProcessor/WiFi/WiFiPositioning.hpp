// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file WiFiPositioning.hpp
/// @brief WiFi Positioning
/// @author R. Lintz (r-lintz@gmx.de) (master thesis)
/// @date 2024-01-08

#pragma once

#include "util/Eigen.hpp"
#include "util/CallbackTimer.hpp"
#include <vector>

#include "internal/Node/Node.hpp"
#include "NodeData/WiFi/WiFiObs.hpp"
#include "NodeData/State/Pos.hpp"
#include "NodeData/State/PosVel.hpp"

#include "Navigation/Math/KeyedKalmanFilter.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/LeastSquares.hpp"

namespace NAV
{
/// @brief Numerically integrates Imu data
class WiFiPositioning : public Node
{
  public:
    /// @brief Default constructor
    WiFiPositioning();
    /// @brief Destructor
    ~WiFiPositioning() override;
    /// @brief Copy constructor
    WiFiPositioning(const WiFiPositioning&) = delete;
    /// @brief Move constructor
    WiFiPositioning(WiFiPositioning&&) = delete;
    /// @brief Copy assignment operator
    WiFiPositioning& operator=(const WiFiPositioning&) = delete;
    /// @brief Move assignment operator
    WiFiPositioning& operator=(WiFiPositioning&&) = delete;

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

  private:
    constexpr static size_t INPUT_PORT_INDEX_WIFI_OBS = 0; ///< @brief WiFiObs
    constexpr static size_t OUTPUT_PORT_INDEX_WIFISOL = 0; ///< @brief WiFiPositioningSolution

    /// @brief Number of states
    uint8_t _numStates = 6;

    /// @brief Number of measurements
    uint8_t _numMeasurements = 1;

    /// @brief Kalman Filter representation - States: 3xVel, 3xPos, (1xBias) - Measurements: 1xDist
    KalmanFilter _kalmanFilter{ _numStates, _numMeasurements }; // TODO: Change to KeyedKalmanFilter

    // --------------------------------------------------------------- Gui -----------------------------------------------------------------

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Amount of wifi input pins
    size_t _nWifiInputPins = 1;

    /// @brief Adds/Deletes Input Pins depending on the variable _nNavInfoPins
    void updateNumberOfInputPins();

    // ------------------------------------------------------------ Algorithm --------------------------------------------------------------

    /// @brief Available Frames
    enum class Frame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        LLA,  ///< Latitude-Longitude-Altitude frame
    };
    /// Frame to calculate the position in
    Frame _frame = Frame::ECEF;

    /// @brief Available Solution Modes
    enum class SolutionMode : int
    {
        LSQ, ///< Least Squares
        KF,  ///< Kalman Filter
    };
    /// @brief Solution Mode
    SolutionMode _solutionMode = SolutionMode::LSQ;

    /// @brief Selection of whether the bias will be additionally estimated
    bool _estimateBias = false;

    /// @brief Selection of whether the solution will be weighted
    bool _weightedSolution = false;

    /// @brief Selection of whether the initial values should always be used or those of the last position
    bool _useInitialValues = false;

    /// @brief State estimated by the positioning algorithm
    struct State
    {
        /// @brief Estimated position in ECEF frame [m]
        Eigen::Vector3d e_position = Eigen::Vector3d::Zero();
        /// @brief Estimated velocity in ECEF frame [m/s]
        Eigen::Vector3d e_velocity = Eigen::Vector3d::Zero();
        /// @brief Estimated bias [m]
        double bias = 0;
    };

    /// @brief State estimated by the algorithm
    State _state;

    /// @brief Initial state
    State _initialState;

    /// @brief Input of mac addresses
    std::vector<std::string> _deviceMacAddresses{ 1 };

    /// @brief Input of positions
    std::vector<Eigen::Vector3d> _devicePositions{ 1 };

    /// @brief Input of biases
    std::vector<double> _deviceBias{ 1, 0.0 };

    /// @brief Input of scales
    std::vector<double> _deviceScale{ 1, 0.0 };

    /// @brief Number of devices
    size_t _numOfDevices = 0;

    /// @brief Device struct
    struct Device
    {
        Eigen::Vector3d position{ Eigen::Vector3d::Zero() }; ///< Position vector
        InsTime time{};                                      ///< Time
        double distance{ 0.0 };                              ///< Distance
        double distanceStd{ 0.0 };                           ///< Standard deviation of distance measurement
    };
    /// @brief Devices which are used for the positioning
    std::vector<Device> _devices{ 1 };

    /// @brief Time when the last prediction was triggered
    InsTime _lastPredictTime;

    /// @brief Receive Function for the WiFi Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvWiFiObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Calculate the position using the least squares method
    /// @return Least Squares solution
    LeastSquaresResult<Eigen::VectorXd, Eigen::MatrixXd> lsqSolution();

    /// @brief Calculate the position
    void kfSolution();

    // ###########################################################################################################

    /// Possible Units for the measurement noise (standard deviation σ or Variance σ²)
    enum class MeasurementNoiseUnit
    {
        meter2, ///< Variance NED [m^2, m^2, m^2]
        meter,  ///< Standard deviation NED [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    MeasurementNoiseUnit _measurementNoiseUnit = MeasurementNoiseUnit::meter;

    /// GUI selection of the process noise (standard deviation σ or Variance σ²)
    double _measurementNoise = 10;

    // ###########################################################################################################

    /// @brief Possible Units for the process noise (standard deviation σ or Variance σ²)
    enum class ProcessNoiseUnit
    {
        meter2, ///< Variance NED [m^2, m^2, m^2]
        meter,  ///< Standard deviation NED [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    ProcessNoiseUnit _processNoiseUnit = ProcessNoiseUnit::meter;

    /// GUI selection of the process noise (standard deviation σ or Variance σ²)
    double _processNoise = 10;

    // ###########################################################################################################

    /// @brief Possible Units for the initial covariance for the position (standard deviation σ or Variance σ²)
    enum class InitCovariancePositionUnit
    {
        meter2, ///< Variance NED [m^2, m^2, m^2]
        meter,  ///< Standard deviation NED [m, m, m]
    };
    /// Gui selection for the Unit of the initial covariance for the position
    InitCovariancePositionUnit _initCovariancePositionUnit = InitCovariancePositionUnit::meter;

    /// GUI selection of the initial covariance diagonal values for position (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovariancePosition{ 100, 100, 100 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the velocity (standard deviation σ or Variance σ²)
    enum class InitCovarianceVelocityUnit
    {
        m2_s2, ///< Variance [m^2/s^2]
        m_s,   ///< Standard deviation [m/s]
    };
    /// Gui selection for the Unit of the initial covariance for the velocity
    InitCovarianceVelocityUnit _initCovarianceVelocityUnit = InitCovarianceVelocityUnit::m_s;

    /// GUI selection of the initial covariance diagonal values for velocity (standard deviation σ or Variance σ²)
    Eigen::Vector3d _initCovarianceVelocity{ 10, 10, 10 };

    // ###########################################################################################################

    /// Possible Units for the initial covariance for the bias (standard deviation σ or Variance σ²)
    enum class InitCovarianceBiasUnit
    {
        meter2, ///< Variance [m^2]
        meter,  ///< Standard deviation [m]
    };
    /// Gui selection for the Unit of the initial covariance for the bias
    InitCovarianceBiasUnit _initCovarianceBiasUnit = InitCovarianceBiasUnit::meter;

    /// GUI selection of the initial covariance diagonal values for bias (standard deviation σ or Variance σ²)
    double _initCovarianceBias = 10;
};
} // namespace NAV