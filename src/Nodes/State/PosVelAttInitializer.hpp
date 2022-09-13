/// @file PosVelAttInitializer.hpp
/// @brief Position, Velocity, Attitude Initializer from GPS and IMU data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-03

#pragma once

#include "internal/Node/Node.hpp"

#include "Navigation/Time/InsTime.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include <limits>

namespace NAV
{
/// Position, Velocity, Attitude Initializer from GPS and IMU data
class PosVelAttInitializer : public Node
{
  public:
    /// @brief Default constructor
    PosVelAttInitializer();
    /// @brief Destructor
    ~PosVelAttInitializer() override;
    /// @brief Copy constructor
    PosVelAttInitializer(const PosVelAttInitializer&) = delete;
    /// @brief Move constructor
    PosVelAttInitializer(PosVelAttInitializer&&) = delete;
    /// @brief Copy assignment operator
    PosVelAttInitializer& operator=(const PosVelAttInitializer&) = delete;
    /// @brief Move assignment operator
    PosVelAttInitializer& operator=(PosVelAttInitializer&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_POS_VEL_ATT = 0; ///< @brief Flow (PosVelAtt)

    /// Index of the input pin for IMU observations
    int _inputPinIdxIMU = -1;
    /// Index of the input pin for GNSS observations
    int _inputPinIdxGNSS = -1;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Add or removes input pins depending on the settings
    void updateInputPins();

    /// Checks whether all Flags are set and writes logs messages
    void finalizeInit();

    /// @brief Receive Imu Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveImuObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Gnss Observations
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveGnssObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Receive Ublox Observations
    /// @param[in] obs Ublox Data
    void receiveUbloxObs(const std::shared_ptr<const UbloxObs>& obs);

    /// @brief Receive Ublox Observations
    /// @param[in] obs RtklibPos Data
    void receiveRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs);

    /// @brief Receive Pos Observations
    /// @param[in] obs Pos Data
    void receivePosObs(const std::shared_ptr<const Pos>& obs);

    /// @brief Receive PosVel Observations
    /// @param[in] obs PosVel Data
    void receivePosVelObs(const std::shared_ptr<const PosVel>& obs);

    /// @brief Receive PosVelAtt Observations
    /// @param[in] obs PosVelAtt Data
    void receivePosVelAttObs(const std::shared_ptr<const PosVelAtt>& obs);

    /// @brief Polls the PVA solution if all is set in the GUI
    /// @param[in] peek Specifies if the data should be peeked
    /// @return The PVA solution
    [[nodiscard]] std::shared_ptr<const NodeData> pollPVASolution(bool peek = false);

    /// Time in [s] to initialize the state
    double _initDuration = 5.0;

    /// Start time of the averageing process
    uint64_t _startTime = 0;

    /// Initialization source for attitude
    enum class AttitudeMode
    {
        BOTH,  ///< Use IMU and GNSS Observations for attitude initialization
        IMU,   ///< Use IMU Observations for attitude initialization
        GNSS,  ///< Use GNSS Observations for attitude initialization
        COUNT, ///< Amount of items in the enum
    };

    /// @brief Converts the enum to a string
    /// @param[in] attitudeMode Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(AttitudeMode attitudeMode);

    /// GUI option to pecify the initialization source for attitude
    AttitudeMode _attitudeMode = AttitudeMode::BOTH;

    /// Whether the GNSS values should be used or we want to override the values manually
    bool _overridePosition = false;
    /// Values to override the Position in [deg, deg, m]
    std::array<float, 3> _lla_overrideValuesPosition = {};
    /// Position Accuracy to achieve in [cm]
    float _positionAccuracyThreshold = 10;
    /// Last position accuracy in [cm] for XYZ or NED
    std::array<float, 3> _lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                                                   std::numeric_limits<float>::infinity(),
                                                   std::numeric_limits<float>::infinity() };

    /// Whether the GNSS values should be used or we want to override the values manually
    bool _overrideVelocity = false;
    /// Values to override the Velocity in [m/s]
    std::array<float, 3> _n_overrideValuesVelocity = {};
    /// Velocity Accuracy to achieve in [cm/s]
    float _velocityAccuracyThreshold = 10;
    /// Last velocity accuracy in [cm/s] for XYZ or NED
    std::array<float, 3> _lastVelocityAccuracy = { std::numeric_limits<float>::infinity(),
                                                   std::numeric_limits<float>::infinity(),
                                                   std::numeric_limits<float>::infinity() };

    /// Count of received attitude measurements
    double _countAveragedAttitude = 0.0;
    /// Averaged Attitude (roll, pitch, yaw) in [rad]
    std::array<double, 3> _averagedAttitude = { 0.0, 0.0, 0.0 };
    /// Whether the IMU values should be used or we want to override the values manually
    std::array<bool, 3> _overrideRollPitchYaw = { false, false, false };
    /// Values to override Roll, Pitch and Yaw with in [deg]
    std::array<float, 3> _overrideValuesRollPitchYaw = {};

    /// Whether the states are initialized (pos, vel, att, messages send)
    std::array<bool, 4> _posVelAttInitialized = { false, false, false, false };

    /// Initialized Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond _n_Quat_b_init;
    /// Position in ECEF coordinates
    Eigen::Vector3d _e_initPosition;
    /// Velocity in navigation coordinates
    Eigen::Vector3d _n_initVelocity;
};

} // namespace NAV
