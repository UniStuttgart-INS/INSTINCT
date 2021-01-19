/// @file State.hpp
/// @brief State Information Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/State/StateData.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV
{
class State : public Node
{
  public:
    /// @brief Default constructor
    State();
    /// @brief Destructor
    ~State() override;
    /// @brief Copy constructor
    State(const State&) = delete;
    /// @brief Move constructor
    State(State&&) = delete;
    /// @brief Copy assignment operator
    State& operator=(const State&) = delete;
    /// @brief Move assignment operator
    State& operator=(State&&) = delete;

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
    constexpr static size_t OutputPortIndex_ImuObs = 1;   ///< @brief Flow (ImuObs)
    constexpr static size_t InputPortIndex_StateData = 0; ///< @brief Object (StateData)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Initialize the State with Imu data
    /// @param[in] state Partial state data
    /// @param[in] linkId Id of the link over which the data is received
    void initAttitude(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Initialize the State with Gnss data
    /// @param[in] state Partial state data
    /// @param[in] linkId Id of the link over which the data is received
    void initPositionVelocity(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Finalize the initialization
    /// @param[in] currentTime The latest time
    void finalizeInit(const InsTime& currentTime);

    /// @brief Update the current State
    /// @param[in] state The new state
    /// @param[in] linkId Id of the link over which the data is received
    void updateState(const std::shared_ptr<NodeData>& state, ax::NodeEditor::LinkId linkId);

    /// The initial vehicle state
    StateData initialState;

    /// The current vehicle state
    StateData currentState;

    bool dynamicStateInit = true;
    std::array<float, 3> initLatLonAlt{ 0, 0, 0 };
    std::array<float, 3> initRollPitchYaw{ 0, 0, 0 };
    std::array<float, 3> initVelocityNED{ 0, 0, 0 };

    double initDuration = 5;
    InsTime averageStartTime;

    double countAveragedAttitude = 0;
    double countAveragedPosition = 0;
    double countAveragedVelocity = 0;
};

} // namespace NAV
