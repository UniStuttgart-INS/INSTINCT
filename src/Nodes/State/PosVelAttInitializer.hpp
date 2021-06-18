/// @file PosVelAttInitializer.hpp
/// @brief Position, Velocity, Attitude Initializer from GPS and IMU data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-03

#pragma once

#include "Nodes/Node.hpp"

#include "util/InsTime.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"

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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(Pin* startPin, Pin* endPin) override;

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 1;  ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_GnssObs = 2; ///< @brief Flow (GnssObs)
    constexpr static size_t InputPortIndex_ImuObs = 0;   ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_GnssObs = 1;  ///< @brief Flow (GnssObs)
    constexpr static size_t InputPortIndex_Position = 2; ///< @brief Matrix
    constexpr static size_t InputPortIndex_Velocity = 3; ///< @brief Matrix
    constexpr static size_t InputPortIndex_Attitude = 4; ///< @brief Matrix

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Checks whether all Flags are set and writes logs messages
    void finalizeInit();

    /// @brief Receive Imu Observations
    /// @param[in] nodeData Imu Data
    /// @param[in] linkId Id of the link over which the data is received
    void receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Gnss Observations
    /// @param[in] nodeData Gnss Data
    /// @param[in] linkId Id of the link over which the data is received
    void receiveGnssObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive Ublox Observations
    /// @param[in] obs Ublox Data
    void receiveUbloxObs(const std::shared_ptr<UbloxObs>& obs);

    /// @brief Receive Ublox Observations
    /// @param[in] obs RtklibPos Data
    void receiveRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs);

    enum class InitFlag
    {
        NOT_CONNECTED,
        CONNECTED,
        INITIALIZED,
    };

    /// Position Flag
    InitFlag determinePosition = InitFlag::NOT_CONNECTED;
    /// Velocity Flag
    InitFlag determineVelocity = InitFlag::NOT_CONNECTED;
    /// Attitude Flag
    InitFlag determineAttitude = InitFlag::NOT_CONNECTED;

    /// Time in [s] to initialize the state
    double initDuration = 5.0;

    /// Start time of the averageing process
    uint64_t startTime = 0;

    /// Position Accuracy to achieve in [cm]
    float positionAccuracyThreshold = 10;
    /// Last position accuracy in [cm] for XYZ or NED
    std::array<float, 3> lastPositionAccuracy = { std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity(),
                                                  std::numeric_limits<float>::infinity() };

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
};

} // namespace NAV