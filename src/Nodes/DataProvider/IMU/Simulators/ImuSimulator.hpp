/// @file ImuSimulator.hpp
/// @brief Imu Observation Simulator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"

#include "util/Eigen.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Gravity/Gravity.hpp"

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

    /// Start position in local navigation coordinates (latitude, longitude, altitude) [rad, rad, m]
    ///
    /// - Fixed, Linear: Start position
    /// - Circular, Helix: Center of the circle
    Eigen::Vector3d startPosition_lla = Eigen::Vector3d::Zero();

    /// Start position in ECEF coordinates in [m]. Will be set at initialization
    Eigen::Vector3d startPosition_e;

    /// Orientation of the vehicle [roll, pitch, yaw] [rad]
    Eigen::Vector3d fixedTrajectoryStartOrientation = Eigen::Vector3d::Zero();

    /// Velocity of the vehicle in local-navigation frame cooridnates in [m/s]
    Eigen::Vector3d linearTrajectoryVelocity_n = Eigen::Vector3d{ 1, 0, 0 };

    /// Horizontal speed of the vehicle in the tangential plane in [m/s]
    double circularTrajectoryHorizontalSpeed = 1.0;

    /// Vertical speed of the vehicle in the tangential plane in [m/s]
    double helicalTrajectoryVerticalSpeed = 1.0;

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

    /// Gravity model selected in the GUI
    GravityModel gravityModel = GravityModel::EGM96;

    /// Apply the coriolis acceleration to the measured accelerations
    bool coriolisAccelerationEnabled = true;

    /// Apply the centrifugal acceleration to the measured accelerations
    bool centrifgalAccelerationEnabled = true;

    /// Apply the Earth rotation rate to the measured angular rates
    bool angularRateEarthRotationEnabled = true;

    /// Apply the transport rate to the measured angular rates
    bool angularRateTransportRateEnabled = true;

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
    /// @param[in] q_ne Rotation quaternion from Earth frame to local-navigation frame
    /// @return v_n in [rad, rad, m]
    Eigen::Vector3d calcVelocity_n(double time, const Eigen::Quaterniond& q_ne);

    /// @brief Calculates the acceleration in local-navigation frame coordinates at the given time depending on the trajectoryType
    /// @param[in] time Time in [s]
    /// @param[in] q_ne Rotation quaternion from Earth frame to local-navigation frame
    /// @return a_n in [rad, rad, m]
    Eigen::Vector3d calcTrajectoryAccel_n(double time, const Eigen::Quaterniond& q_ne);

    /// @brief Calculates ω_ip_p, the gyroscope measurement (turn rate of the platform with respect to the inertial system expressed in platform coordinates)
    /// @param[in] velocity_n Velocity in local-navigation frame coordinates [m/s]
    /// @param[in] acceleration_n Acceleration in local-navigation frame coordinates [m/s^2]
    /// @param[in] rollPitchYaw Gimbal angles (roll, pitch, yaw) [rad]
    /// @param[in] q_bn Rotation quaternion from local-navigation frame to the body frame
    /// @param[in] omega_ie_n ω_ie_n Earth rotation rate in local-navigation coordinates
    /// @param[in] omega_en_n ω_en_n Transport rate in local-navigation coordinates
    /// @return ω_ip_p [rad/s]
    Eigen::Vector3d calcOmega_ip_p(const Eigen::Vector3d& velocity_n,
                                   const Eigen::Vector3d& acceleration_n,
                                   const Eigen::Vector3d& rollPitchYaw,
                                   const Eigen::Quaterniond& q_bn,
                                   const Eigen::Vector3d& omega_ie_n,
                                   const Eigen::Vector3d& omega_en_n);
};

} // namespace NAV
