/// @file PosVelAttInitializer.hpp
/// @brief Position, Velocity, Attitude Initializer from GPS and IMU data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-03

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Time/InsTime.hpp"

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
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OutputPortIndex_PosVelAtt = 0; ///< @brief Flow (PosVelAtt)
    constexpr static size_t InputPortIndex_ImuObs = 0;     ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_GnssObs = 1;    ///< @brief Flow (GnssObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Checks whether all Flags are set and writes logs messages
    void finalizeInit();

    /// @brief Receive Imu Observations
    /// @param[in] nodeData Imu Data
    /// @param[in] linkId Id of the link over which the data is received
    void receiveImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Gnss Observations
    /// @param[in] nodeData Gnss Data
    /// @param[in] linkId Id of the link over which the data is received
    void receiveGnssObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Ublox Observations
    /// @param[in] obs Ublox Data
    void receiveUbloxObs(const std::shared_ptr<const UbloxObs>& obs);

    /// @brief Receive Ublox Observations
    /// @param[in] obs RtklibPos Data
    void receiveRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs);

    /// @brief Receive PosVelAtt Observations
    /// @param[in] obs PosVelAtt Data
    void receivePosVelAttObs(const std::shared_ptr<const PosVelAtt>& obs);

    /// Time in [s] to initialize the state
    double initDuration = 5.0;

    /// Start time of the averageing process
    uint64_t startTime = 0;

    /// Initialization source for attitude
    enum AttitudeMode
    {
        AttitudeMode_BOTH, ///< Use IMU and GNSS Observations for attitude initialization
        AttitudeMode_IMU,  ///< Use IMU Observations for attitude initialization
        AttitudeMode_GNSS, ///< Use GNSS Observations for attitude initialization
    };

    /// GUI option to pecify the initialization source for attitude
    AttitudeMode attitudeMode = AttitudeMode_BOTH;

    /// Whether the GNSS values should be used or we want to override the values manually
    bool overridePosition = false;
    /// Values to override the Position in [deg, deg, m]
    std::array<float, 3> overrideValuesPosition_lla = {};
    /// Position Accuracy to achieve in [cm]
    float positionAccuracyThreshold = 10;
    /// Last position accuracy in [cm] for XYZ or NED
    std::array<float, 3> lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity() };

    /// Whether the GNSS values should be used or we want to override the values manually
    bool overrideVelocity = false;
    /// Values to override the Velocity in [m/s]
    std::array<float, 3> overrideValuesVelocity_n = {};
    /// Velocity Accuracy to achieve in [cm/s]
    float velocityAccuracyThreshold = 10;
    /// Last velocity accuracy in [cm/s] for XYZ or NED
    std::array<float, 3> lastVelocityAccuracy = { std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity() };

    /// Count of received attitude measurements
    double countAveragedAttitude = 0.0;
    /// Averaged Attitude (roll, pitch, yaw) in [rad]
    std::array<double, 3> averagedAttitude = { 0.0, 0.0, 0.0 };
    /// Whether the IMU values should be used or we want to override the values manually
    std::array<bool, 3> overrideRollPitchYaw = { false, false, false };
    /// Values to override Roll, Pitch and Yaw with in [deg]
    std::array<float, 3> overrideValuesRollPitchYaw = {};

    /// Whether the states are initialized (pos, vel, att, messages send)
    std::array<bool, 4> posVelAttInitialized = { false, false, false, false };

    /// Initialized Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond q_nb_init;
    /// Position in ECEF coordinates
    Eigen::Vector3d p_ecef_init;
    /// Velocity in navigation coordinates
    Eigen::Vector3d v_n_init;
};

} // namespace NAV
