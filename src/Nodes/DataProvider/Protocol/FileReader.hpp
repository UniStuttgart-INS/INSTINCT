/// @file FileReader.hpp
/// @brief Abstract File Reader class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <map>
#include <fstream>
#include <optional>

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
    /// @param[in] options Program options string map
    explicit FileReader(const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    FileReader() = default;
    /// @brief Destructor
    virtual ~FileReader() = default;

    /// @brief Virtual Function to determine the File Type
    /// @return The File path which was recognized
    [[nodiscard]] virtual FileType determineFileType() = 0;

    /// Path to log file
    std::string path;
    /// File stream to read the file
    std::ifstream filestream;
    /// File Type
    FileType fileType = FileType::NONE;
    /// Start of the data in the file
    std::streampos dataStart = 0;
};

} // namespace NAV
