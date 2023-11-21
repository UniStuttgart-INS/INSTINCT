// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ImuSimulator.hpp
/// @brief Imu Observation Simulator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author N. Stahl (Hiwi: Rose figure trajectory type)
/// @date 2023-07-18

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Eigen.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/CubicSpline.hpp"
#include "internal/gui/widgets/TimeEdit.hpp"
#include "internal/gui/widgets/PositionInput.hpp"

#include "NodeData/General/CsvData.hpp"

#include <array>

namespace NAV
{
/// Imu Observation Simulator
class ImuSimulator : public Imu
{
  public:
    /// @brief Default constructor
    ImuSimulator();
    /// @brief Destructor
    ~ImuSimulator() override;
    /// @brief Copy constructor
    ImuSimulator(const ImuSimulator&) = delete;
    /// @brief Move constructor
    ImuSimulator(ImuSimulator&&) = delete;
    /// @brief Copy assignment operator
    ImuSimulator& operator=(const ImuSimulator&) = delete;
    /// @brief Move assignment operator
    ImuSimulator& operator=(ImuSimulator&&) = delete;

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

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t INPUT_PORT_INDEX_CSV = 0;          ///< @brief Object (CsvData)
    constexpr static size_t OUTPUT_PORT_INDEX_IMU_OBS = 0;     ///< @brief Flow (ImuObs)
    constexpr static size_t OUTPUT_PORT_INDEX_POS_VEL_ATT = 1; ///< @brief Flow (PosVelAtt)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls the next simulated data
    /// @param[in] pinIdx Index of the pin the data is requested on
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollImuObs(size_t pinIdx, bool peek);

    /// @brief Polls the next simulated data
    /// @param[in] pinIdx Index of the pin the data is requested on
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollPosVelAtt(size_t pinIdx, bool peek);

    /// @brief Checks the selected stop condition
    /// @param[in] time Current simulation time
    /// @param[in] lla_position Current position
    /// @return True if it should be stopped
    bool checkStopCondition(double time, const Eigen::Vector3d& lla_position);

    // ###########################################################################################################

    /// Types where the start time should be pulled from
    enum class StartTimeSource
    {
        CustomTime,          ///< Custom time selected by the user
        CurrentComputerTime, ///< Gets the current computer time as start time
    };

    /// Source for the start time, selected in the GUI
    StartTimeSource _startTimeSource = StartTimeSource::CustomTime;

    /// Time Format to input the start time with
    gui::widgets::TimeEditFormat _startTimeEditFormat;

    /// Global starttime
    InsTime _startTime{ 2000, 1, 1, 0, 0, 0 };

    // ###########################################################################################################

    /// Frequency to sample the IMU with in [Hz]
    double _imuFrequency = 100;
    /// Frequency to sample the position with in [Hz]
    double _gnssFrequency = 5;

    // ###########################################################################################################

    /// Types of Trajectories available for simulation
    enum class TrajectoryType
    {
        Fixed,      ///< Static position without movement
        Linear,     ///< Linear movement with constant velocity
        Circular,   ///< Circular path
        Csv,        ///< Get the input from the CsvData pin
        RoseFigure, ///< Movement along a mathmatical rose figure
        COUNT,      ///< Amount of items in the enum
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(TrajectoryType value);

    /// Selected trajectory type in the GUI
    TrajectoryType _trajectoryType = TrajectoryType::Fixed;

    /// Start position in local navigation coordinates (latitude, longitude, altitude) [rad, rad, m]
    ///
    /// - Fixed, Linear: Start position
    /// - Circular: Center of the circle
    gui::widgets::PositionWithFrame _startPosition;

    /// Orientation of the vehicle [roll, pitch, yaw] [rad]
    Eigen::Vector3d _fixedTrajectoryStartOrientation = Eigen::Vector3d::Zero();

    /// Start Velocity of the vehicle in local-navigation frame cooridnates in [m/s]
    Eigen::Vector3d _n_linearTrajectoryStartVelocity = Eigen::Vector3d{ 1, 0, 0 };

    /// Harmonic Oscillation Frequency on the circular trajectory [cycles/revolution]
    int _circularHarmonicFrequency = 0;

    /// Harmonic Oscillation Amplitude Factor of the circle radius [-]
    double _circularHarmonicAmplitudeFactor = 0.1;

    /// Possible directions for the circular trajectory
    enum class Direction
    {
        CW,    ///< Clockwise
        CCW,   ///< Counterclockwise
        COUNT, ///< Amount of items in the enum
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(Direction value);

    /// In the GUI selected direction of the circular trajectory (used by circular and rose figure)
    Direction _trajectoryDirection = Direction::CCW;

    /// In the GUI selected origin angle of the circular trajectory in [rad]
    double _trajectoryRotationAngle = 0.0;

    /// Horizontal speed of the vehicle in the tangential plane in [m/s] (used by circular and rose figure)
    double _trajectoryHorizontalSpeed = 10.0;

    /// Vertical speed of the vehicle in the tangential plane in [m/s] (used by circular and rose figure)
    double _trajectoryVerticalSpeed = 0.0;

    /// In the GUI selected radius of the circular trajectory (used by circular and rose figure)
    double _trajectoryRadius = 50.0;

    /// In the GUI selected numerator of petals (2*k for even k, k for uneven k) of the rose figure
    int _rosePetNum = 2;

    /// In the GUI selected denominator of petals (2*k for even k, k for uneven k) of the rose figure
    int _rosePetDenom = 1;

    /// Maxmimum step length for the spline points for the rose figure [m]. Points will be spaced between [L/3 L]
    double _roseStepLengthMax = 0.1;

    /// Simulation duration needed for the rose figure
    double _roseSimDuration = 0.0;

    // ###########################################################################################################

    /// Possible stop conditions for the simulation
    enum StopCondition
    {
        Duration,                 ///< Time Duration
        DistanceOrCirclesOrRoses, ///< Distance for Linear trajectory / Circle count for Circular / Count for rose figure trajectory
    };

    /// Condition which has to be met to stop the simulation
    StopCondition _simulationStopCondition = StopCondition::Duration;

    /// Duration to simulate in [s]
    double _simulationDuration = 5 * 60;

    /// Duration from the CSV file in [s]
    double _csvDuration = 0;

    /// Distance in [m] to the start position to stop the simulation
    double _linearTrajectoryDistanceForStop = 100;

    /// Amount of circles to simulate before stopping
    double _circularTrajectoryCircleCountForStop = 1.0;

    /// Amount of rose figures to simulate before stopping
    double _roseTrajectoryCountForStop = 1.0;
    // ###########################################################################################################

    /// Gravitation model selected in the GUI
    GravitationModel _gravitationModel = GravitationModel::EGM96;

    /// Apply the coriolis acceleration to the measured accelerations
    bool _coriolisAccelerationEnabled = true;

    /// Apply the centrifugal acceleration to the measured accelerations
    bool _centrifgalAccelerationEnabled = true;

    /// Apply the Earth rotation rate to the measured angular rates
    bool _angularRateEarthRotationEnabled = true;

    /// Apply the transport rate to the measured angular rates
    bool _angularRateTransportRateEnabled = true;

    // ###########################################################################################################

    /// @brief Get the Time from a CSV line
    /// @param[in] line Line with data from the csv
    /// @param[in] description Description of the data
    /// @return InsTime or empty time if data not found
    [[nodiscard]] InsTime getTimeFromCsvLine(const CsvData::CsvLine& line, const std::vector<std::string>& description) const;

    /// @brief Get the Position from a CSV line
    /// @param[in] line Line with data from the csv
    /// @param[in] description Description of the data
    /// @return Position in ECEF coordinates in [m] or NaN if data not found
    [[nodiscard]] Eigen::Vector3d e_getPositionFromCsvLine(const CsvData::CsvLine& line, const std::vector<std::string>& description) const;

    /// @brief Get the Attitude quaternion n_quat_b from a CSV line
    /// @param[in] line Line with data from the csv
    /// @param[in] description Description of the data
    /// @return Attitude quaternion n_quat_b or NaN if data not found
    static Eigen::Quaterniond n_getAttitudeQuaternionFromCsvLine_b(const CsvData::CsvLine& line, const std::vector<std::string>& description);

    /// Assign a variable that holds the Spline information
    struct
    {
        double sampleInterval = 0.1; ///< Spline sample interval
        CubicSpline x;               ///< ECEF X Position [m]
        CubicSpline y;               ///< ECEF Y Position [m]
        CubicSpline z;               ///< ECEF Z Position [m]
        CubicSpline roll;            ///< Roll angle [rad]
        CubicSpline pitch;           ///< Pitch angle [rad]
        CubicSpline yaw;             ///< Yaw angle [rad]
    } _splines;

    /// @brief Initializes the spline values
    /// @return True if everything succeeded
    bool initializeSplines();

    /// Counter to calculate the IMU update time
    uint64_t _imuUpdateCnt = 0.0;
    /// Counter to calculate the GNSS update time
    uint64_t _gnssUpdateCnt = 0.0;

    /// Update rate for the internal solution of linear movement in [Hz]
    static constexpr double INTERNAL_LINEAR_UPDATE_FREQUENCY = 1000;

    /// Last time the IMU message was calculated in [s]
    double _imuLastUpdateTime = 0.0;
    /// Last time the GNSS message was calculated in [s]
    double _gnssLastUpdateTime = 0.0;
    /// Last calculated position for the IMU in linear mode for iterative calculations as latitude, longitude, altitude [rad, rad, m]
    Eigen::Vector3d _lla_imuLastLinearPosition = Eigen::Vector3d::Zero();
    /// Last calculated position for the GNSS in linear mode for iterative calculations as latitude, longitude, altitude [rad, rad, m]
    Eigen::Vector3d _lla_gnssLastLinearPosition = Eigen::Vector3d::Zero();

    /// @brief Calculates the flight angles (roll, pitch, yaw)
    /// @param[in] time Time in [s]
    /// @return Roll, pitch, yaw in [rad]
    [[nodiscard]] std::array<double, 3> calcFlightAngles(double time) const;

    /// @brief Calculates the position in latLonAlt at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @return LatLonAlt in [rad, rad, m]
    [[nodiscard]] Eigen::Vector3d lla_calcPosition(double time) const;

    /// @brief Calculates the velocity in local-navigation frame coordinates at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @param[in] n_Quat_e Rotation quaternion from Earth frame to local-navigation frame
    /// @return n_velocity in [rad, rad, m]
    [[nodiscard]] Eigen::Vector3d n_calcVelocity(double time, const Eigen::Quaterniond& n_Quat_e) const;

    /// @brief Calculates the acceleration in local-navigation frame coordinates at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @param[in] n_Quat_e Rotation quaternion from Earth frame to local-navigation frame
    /// @param[in] lla_position Current position as latitude, longitude, altitude [rad, rad, m]
    /// @param[in] n_velocity Velocity in local-navigation frame coordinates [m/s]
    /// @return n_accel in [rad, rad, m]
    [[nodiscard]] Eigen::Vector3d n_calcTrajectoryAccel(double time, const Eigen::Quaterniond& n_Quat_e, const Eigen::Vector3d& lla_position, const Eigen::Vector3d& n_velocity) const;

    /// @brief Calculates ω_ip_p, the gyroscope measurement (turn rate of the platform with respect to the inertial system expressed in NED coordinates)
    /// @param[in] time Time in [s]
    /// @param[in] rollPitchYaw Gimbal angles (roll, pitch, yaw) [rad]
    /// @param[in] n_Quat_b Rotation quaternion from body frame to the local-navigation frame
    /// @param[in] n_omega_ie ω_ie_n Earth rotation rate in local-navigation coordinates
    /// @param[in] n_omega_en ω_en_n Transport rate in local-navigation coordinates
    /// @return ω_ip_p [rad/s]
    [[nodiscard]] Eigen::Vector3d n_calcOmega_ip(double time,
                                                 const Eigen::Vector3d& rollPitchYaw,
                                                 const Eigen::Quaterniond& n_Quat_b,
                                                 const Eigen::Vector3d& n_omega_ie,
                                                 const Eigen::Vector3d& n_omega_en) const;
};

} // namespace NAV
