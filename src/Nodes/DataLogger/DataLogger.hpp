/// @file DataLogger.hpp
/// @brief Abstract Data Logger class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

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
  public:
    /// File Type
    enum FileType
    {
        /// Not specified
        NONE,
        /// Binary data
        BINARY,
        /// Ascii text data
        ASCII
    };

    /// @brief Copy constructor
    DataLogger(const DataLogger&) = delete;
    /// @brief Move constructor
    DataLogger(DataLogger&&) = delete;
    /// @brief Copy assignment operator
    DataLogger& operator=(const DataLogger&) = delete;
    /// @brief Move assignment operator
    DataLogger& operator=(DataLogger&&) = delete;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Logger
    /// @param[in] options Program options string map
    DataLogger(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    DataLogger() = default;

    /// @brief Destructor
    ~DataLogger() override;

    /// Path to the log file
    std::string path;

    /// File stream to write the file
    std::ofstream filestream;

    /// File Type
    FileType fileType = FileType::NONE;
};

} // namespace NAV
