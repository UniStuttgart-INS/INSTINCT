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
    constexpr static size_t OutputPortIndex_NodeData = 1; ///< @brief Flow (NodeData)
    constexpr static size_t InputPortIndex_DemoNode = 0;  ///< @brief Flow (Demo)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

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

    /// Time in [s] to initialize the state
    double initDuration = 5.0;

    /// Start time of the averageing process
    InsTime startTime;

    /// Count of received attitude measurements
    double countAveragedAttitude = 0.0;
    /// Count of received position measurements
    double countAveragedPosition = 0.0;
    /// Count of received velocity measurements
    double countAveragedVelocity = 0.0;
};

} // namespace NAV
