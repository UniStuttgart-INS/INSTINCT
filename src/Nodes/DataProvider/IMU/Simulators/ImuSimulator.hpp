/// @file ImuSimulator.hpp
/// @brief Imu Observation Simulator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Eigen.hpp"
#include "util/InsTime.hpp"

#include "internal/gui/widgets/TimeEdit.hpp"

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
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 0;    ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_PosVelAtt = 1; ///< @brief Flow (PosVelAtt)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls the next simulated data
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollImuObs(bool peek = false);

    /// @brief Polls the next simulated data
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollPosVelAtt(bool peek = false);

    // ###########################################################################################################

    /// Types where the start time should be pulled from
    enum class StartTimeSource
    {
        CustomTime,          ///< Custom time selected by the user
        CurrentComputerTime, ///< Gets the current computer time as start time
    };

    /// Source for the start time, selected in the GUI
    StartTimeSource startTimeSource = StartTimeSource::CustomTime;

    /// Time Format to input the start time with
    gui::widgets::TimeEditFormat startTimeEditFormat = gui::widgets::TimeEditFormat::YMDHMS;

    /// Global starttime
    InsTime startTime{ 2000, 1, 1, 0, 0, 0 };

    // ###########################################################################################################

    /// Frequency to sample the IMU with in [Hz]
    double imuFrequency = 200;
    /// Frequency to sample the position with in [Hz]
    double gnssFrequency = 5;

    // ###########################################################################################################

    /// Types of Trajectories available for simulation
    enum class TrajectoryType
    {
        Fixed,    ///< Static position without movement
        Linear,   ///< Linear movement with constant velocity
        Circular, ///< Circular path in the horizontal plane
        Helix,    ///< Circular path with velocity in normal direction of the circle plane
        COUNT,    ///< Amount of items in the enum
        // Spline,   ///< Path which follows a spline trajectory // TODO: Implement ImuSimulator Spline Trajectory
    };
    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(TrajectoryType value);

    /// Selected trajectory type in the GUI
    TrajectoryType trajectoryType = TrajectoryType::Fixed;

    /// Position in local navigation coordinates (latitude, longitude, altitude) [rad, rad, m]
    ///
    /// - Fixed, Linear: Start position
    /// - Circular, Helix: Center of the circle
    Eigen::Vector3d startPosition_lla = Eigen::Vector3d::Zero();

    /// Orientation of the vehicle [roll, pitch, yaw] [rad]
    Eigen::Vector3d startOrientation = Eigen::Vector3d::Zero();

    /// Velocity of the vehicle in [m/s]
    ///
    /// - Linear: Velocity of the vehicle in NED coordinates
    /// - Circular: Horizontal velocity in the north component
    /// - Helix: Horizontal velocity in the north component, vertical in down component
    Eigen::Vector3d velocity_n = Eigen::Vector3d::Zero();

    /// In the GUI selected radius of the circular/helix trajectory
    double circularTrajectoryRadius = 50.0;

    /// In the GUI selected origin angle of the circular/helix trajectory in [rad]
    double circularTrajectoryOriginAngle = 0.0;

    /// Possible directions for the circular/helix trajectory
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

    /// In the GUI selected direction of the circular/helix trajectory
    Direction circularTrajectoryDirection = Direction::CCW;

    // ###########################################################################################################

    /// Possible stop conditions for the simulation
    enum StopCondition
    {
        Duration,          ///< Time Duration
        DistanceOrCircles, ///< Distance for Linear trajectory / Circle count for Circular/Helix trajectory
    };

    /// Condition which has to be met to stop the simulation
    StopCondition simulationStopCondition = StopCondition::Duration;

    /// Duration to simulate in [s]
    double simulationDuration = 5 * 60;

    /// Distance in [m] to the start position to stop the simulation
    double linearTrajectoryDistanceForStop = 100;

    /// Amount of circles to simulate before stopping
    double circularTrajectoryCircleCountForStop = 1.0;

    // ###########################################################################################################

    /// Time to send the next IMU message in [s]
    double imuUpdateTime = 0.0;
    /// Time to send the next GNSS message in [s]
    double gnssUpdateTime = 0.0;

    /// @brief Calculates the position in latLonAlt at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @return LatLonAlt in [rad, rad, m]
    Eigen::Vector3d calcPosition_lla(double time);

    /// @brief Calculates the velocity in local-navigation frame coordinates at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @return v_n in [rad, rad, m]
    Eigen::Vector3d calcVelocity_n(double time);

    /// @brief Calculates the acceleration in local-navigation frame coordinates at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @return a_n in [rad, rad, m]
    Eigen::Vector3d calcAccel_n(double time);
};

} // namespace NAV
