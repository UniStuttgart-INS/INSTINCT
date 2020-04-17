/**
 * @file UbloxDataLogger.hpp
 * @brief Data Logger for Ublox observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-17
 */

#pragma once

#include "../DataLogger.hpp"

namespace NAV
{
/// Data Logger for Ublox observations
class UbloxDataLogger : public DataLogger
{
  public:
    /**
     * @brief Construct a new Data Logger object
     * 
     * @param[in] name Name of the Logger
     * @param[in, out] options Program options string list
     */
    UbloxDataLogger(std::string name, std::deque<std::string>& options);

    /// Default destructor
    virtual ~UbloxDataLogger();

    /**
     * @brief Write Ublox Observation to the file
     * 
     * @param[in] obs The received observation
     * @param[in, out] userData User data specified when registering the callback
     * @retval NavStatus Indicates whether the write was successfull.
     */
    static NavStatus writeObservation(std::shared_ptr<NodeData> obs, std::shared_ptr<Node> userData);
};

} // namespace NAV
