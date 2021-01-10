/// @file TimeSynchronizer.hpp
/// @brief Class to Synchronize Different Data Providers to the same Time base
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"

namespace NAV
{
/// Class to Synchronize Different Data Providers to the same Time base
class TimeSynchronizer : public Node
{
  public:
    /// @brief Default constructor
    TimeSynchronizer();
    /// @brief Destructor
    ~TimeSynchronizer() override;
    /// @brief Copy constructor
    TimeSynchronizer(const TimeSynchronizer&) = delete;
    /// @brief Move constructor
    TimeSynchronizer(TimeSynchronizer&&) = delete;
    /// @brief Copy assignment operator
    TimeSynchronizer& operator=(const TimeSynchronizer&) = delete;
    /// @brief Move assignment operator
    TimeSynchronizer& operator=(TimeSynchronizer&&) = delete;

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

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

  private:
    constexpr static size_t OutputPortIndex_TimeSynchronizer = 0; ///< @brief Delegate
    constexpr static size_t OutputPortIndex_ObsToSync = 1;        ///< @brief Flow
    constexpr static size_t InputPortIndex_ObsToSync = 0;         ///< @brief Flow
    constexpr static size_t InputPortIndex_InsObs = 1;            ///< @brief Flow (InsObs)

    /// @brief Gets the gps time
    /// @param[in] nodeData InsObs to process
    /// @param[in] linkId Id of the link over which the data is received
    void syncTime(std::shared_ptr<NodeData> nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Updates Observations with gps time and calls callbacks
    /// @param[in] nodeData Observation to process
    /// @param[in] linkId Id of the link over which the data is received
    void syncObs(std::shared_ptr<NodeData> nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Polls data from the input port and tries to sync it
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    std::shared_ptr<NodeData> pollData(bool peek);

    /// @brief Updates ImuObs Observations with gps time and calls callbacks
    /// @param[in] obs ImuObs to process
    /// @return True if the time was updated
    bool syncImuObs(std::shared_ptr<ImuObs> obs);

    /// @brief Updates Kvh Observations with gps time and calls callbacks
    /// @param[in] obs KvhObs to process
    /// @return True if the time was updated
    bool syncKvhObs(std::shared_ptr<KvhObs> obs);

    /// Selected Port Type in the Gui
    int selectedPortType = 1;

    /// Flag whether to use the provided start time or wait for a signal on the input port
    bool useFixedStartTime = false;

    int gpsCycle = 2;
    int gpsWeek = 38;
    float gpsToW = 259200;

    /// GPS Time when the sync happened
    std::optional<InsTime> startupGpsTime;

    /// KVH specific variable
    std::optional<uint8_t> prevSequenceNumber;
    /// Time Sync depends on Imu Startup Time
    std::optional<uint64_t> startupImuTime;
};

} // namespace NAV
