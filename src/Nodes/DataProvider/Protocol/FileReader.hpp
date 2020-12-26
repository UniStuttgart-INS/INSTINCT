/// @file FileReader.hpp
/// @brief Abstract File Reader class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <vector>
#include <map>
#include <fstream>

#include "util/InsTime.hpp"

namespace NAV
{
/// Abstract File Reader class
class FileReader
{
  public:
    /// File Type Enumeration
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
    FileReader(const FileReader&) = delete;
    /// @brief Move constructor
    FileReader(FileReader&&) = delete;
    /// @brief Copy assignment operator
    FileReader& operator=(const FileReader&) = delete;
    /// @brief Move assignment operator
    FileReader& operator=(FileReader&&) = delete;

  protected:
    /// @brief Constructor
    /// @param[in] name Name of the Node
    /// @param[in] options Program options string map
    explicit FileReader(std::string name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    FileReader() = default;
    /// @brief Destructor
    virtual ~FileReader() = default;

    /// @brief Initialize the file reader
    void initialize();

    /// @brief Moves the read cursor to the start
    void resetReader();

    /// @brief Virtual Function to determine the File Type
    /// @return The File path which was recognized
    [[nodiscard]] virtual FileType determineFileType();

    /// @brief Virtual Function to read the Header of a file
    virtual void readHeader();

    /// Path to log file
    std::string path;
    /// File stream to read the file
    std::ifstream filestream;
    /// File Type
    FileType fileType = FileType::NONE;
    /// Start of the data in the file
    std::streampos dataStart = 0;

    /// Header Columns of a CSV file
    std::vector<std::string> columns;

    /// Lower Time Limit of data to read
    InsTime lowerLimit{ InsTime_MJD(0, 0) };
    /// Upper Time Limit of data to read
    InsTime upperLimit;

  private:
    /// Name of the parent node
    const std::string parentNodeName;
};

} // namespace NAV
