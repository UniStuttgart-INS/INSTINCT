/**
 * @file FileReader.hpp
 * @brief Abstract File Reader class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-16
 */

#pragma once

#include <string>
#include <deque>
#include <fstream>
#include <optional>

namespace NAV
{
/// Abstract File Reader class
class FileReader
{
  public:
    // File Type
    enum FileType
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        ASCII   ///< Ascii text data
    };

    FileReader(const FileReader&) = delete;            ///< Copy constructor
    FileReader(FileReader&&) = delete;                 ///< Move constructor
    FileReader& operator=(const FileReader&) = delete; ///< Copy assignment operator
    FileReader& operator=(FileReader&&) = delete;      ///< Move assignment operator

  protected:
    /**
     * @brief Construct a new File Reader object
     * 
     * @param[in, out] options Program options string list
     */
    explicit FileReader(std::deque<std::string>& options);

    FileReader() = default;          ///< Default constructor
    virtual ~FileReader() = default; ///< Destructor

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
