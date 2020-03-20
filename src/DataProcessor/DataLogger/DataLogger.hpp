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

#include "DataProcessor/DataProcessor.hpp"
#include "DataProvider/InsObs.hpp"

namespace NAV
{
class DataLogger : public DataProcessor
{
  protected:
    /**
     * @brief Construct a new Data Logger object
     * 
     * @param[in] name Name of the Logger
     * @param[in] path Path to the log file
     * @param[in] isBinary Flag if the logfile is a binary file
     */
    DataLogger(std::string name, std::string path, bool isBinary);

    /// Default destructor
    ~DataLogger();

    /**
     * @brief Initialize the File
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize() override;

    /**
     * @brief Deinitialize the file
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize() override;

    /// Path to the log file
    const std::string path;

    /// Flag if the logfile is a binary file
    const bool isBinary;

    /// File stream to write the file
    std::ofstream filestream;
};

} // namespace NAV
