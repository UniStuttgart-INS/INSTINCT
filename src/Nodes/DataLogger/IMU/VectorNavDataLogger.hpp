/**
 * @file VectorNavDataLogger.hpp
 * @brief Data Logger for VectorNav observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-17
 */

#pragma once

#include "../DataLogger.hpp"

#include <any>
#include "NodeData/IMU/VectorNavObs.hpp"

namespace NAV
{
/// Data Logger for VectorNav observations
class VectorNavDataLogger : public DataLogger
{
  public:
    /**
     * @brief Construct a new Data Logger object
     * 
     * @param[in] name Name of the Logger
     * @param[in, out] options Program options string list
     */
    VectorNavDataLogger(std::string name, std::deque<std::string>& options);

    /// Default destructor
    virtual ~VectorNavDataLogger();

    /**
     * @brief Write VectorNav Observation to the file
     * 
     * @param[in] observation The received observation
     * @param[in] userData User data specified when registering the callback
     * @retval NavStatus Indicates whether the write was successfull.
     */
    static NavStatus writeObservation(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);
};

} // namespace NAV
