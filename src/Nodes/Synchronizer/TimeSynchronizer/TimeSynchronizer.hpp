/**
 * @file TimeSynchronizer.hpp
 * @brief Class to Synchronize Different Data Providers to the same Time base
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-21
 */

#pragma once

#include "Nodes/Node.hpp"

#include <optional>

namespace NAV
{
/// Class to Synchronize Different Data Providers to the same Time base
class TimeSynchronizer : public Node
{
  public:
    /**
     * @brief Construct a new Time Synchronizer object
     * 
     * @param[in] name Name of the Object
     * @param[in, out] options Program options string list
     */
    TimeSynchronizer(std::string name, std::deque<std::string>& options);

    /// Default Destructor
    virtual ~TimeSynchronizer();

    /**
     * @brief Gets the gps time from an UbloxSensor
     * 
     * @param[in] observation UbloxObs to process
     * @param[in] userData Pointer to the Node object
     * @retval NavStatus Indicates whether the sync was successfull
     */
    static NavStatus syncUbloxSensor(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);

    /**
     * @brief Updates VectorNav Observations with gps time
     * 
     * @param[in] observation VectorNavObs to process
     * @param[in] userData Pointer to the Node object
     * @retval NavStatus Indicates whether the sync was successfull
     */
    static NavStatus syncVectorNavSensor(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);

  private:
    bool useFixedStartTime = false;

    std::optional<InsTime> startupGpsTime;
    std::optional<uint64_t> startupImuTime;
};

} // namespace NAV
