/**
 * @file DataLogger.hpp
 * @brief Abstract Data Logger class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-16
 */

#pragma once

#include <string>
#include <deque>
#include <fstream>
#include <memory>

#include "Nodes/Node.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
class DataLogger : public Node
{
  public:
    // File Type
    enum FileType
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        ASCII   ///< Ascii text data
    };

    DataLogger(const DataLogger&) = delete;            ///< Copy constructor
    DataLogger(DataLogger&&) = delete;                 ///< Move constructor
    DataLogger& operator=(const DataLogger&) = delete; ///< Copy assignment operator
    DataLogger& operator=(DataLogger&&) = delete;      ///< Move assignment operator

  protected:
    /**
     * @brief Construct a new Data Logger object
     * 
     * @param[in] name Name of the Logger
     * @param[in, out] options Program options string list
     */
    DataLogger(const std::string& name, std::deque<std::string>& options);

    /// Default constructor
    DataLogger() = default;

    /// Destructor
    ~DataLogger() override;

    /// Path to the log file
    std::string path;

    /// File stream to write the file
    std::ofstream filestream;

    /// File Type
    FileType fileType = FileType::NONE;
};

} // namespace NAV
