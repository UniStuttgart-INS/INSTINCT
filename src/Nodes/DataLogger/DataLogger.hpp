/**
 * @file DataLogger.hpp
 * @brief Abstract Data Logger class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-16
 */

#pragma once

#include <string>
#include <fstream>
#include <memory>

#include "Nodes/Node.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
class DataLogger : public Node
{
  protected:
    /**
     * @brief Construct a new Data Logger object
     * 
     * @param[in] name Name of the Logger
     * @param[in, out] options Program options string list
     */
    DataLogger(std::string name, std::deque<std::string>& options);

    /// Default destructor
    ~DataLogger();

    /// Path to the log file
    std::string path;

    /// Flag if the logfile is a binary file
    bool isBinary;

    /// File stream to write the file
    std::ofstream filestream;
};

} // namespace NAV
