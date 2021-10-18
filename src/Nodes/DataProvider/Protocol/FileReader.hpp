/// @file FileReader.hpp
/// @brief Abstract File Reader class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "util/InsTime.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
/// Abstract File Reader class
class FileReader
{
  public:
    /// File Type Enumeration
    enum FileType
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        CSV,    ///< Ascii text data
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
    /// @brief Default constructor
    FileReader() = default;
    /// @brief Destructor
    virtual ~FileReader() = default;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j);

    /// @brief Initialize the file reader
    bool initialize();

    /// @brief Deinitialize the file reader
    void deinitialize();

    /// @brief Moves the read cursor to the start
    void resetReader();

    /// @brief Virtual Function to determine the File Type
    /// @return The File path which was recognized
    [[nodiscard]] virtual FileType determineFileType();

    /// @brief Virtual Function to read the Header of a file
    virtual void readHeader();

    /// Path to the file
    std::string path;
    /// File stream to read the file
    std::ifstream filestream;
    /// File Type
    FileType fileType = FileType::NONE;
    /// Start of the data in the file
    std::streampos dataStart = 0;

    /// Header Columns of a CSV file
    std::vector<std::string> headerColumns;
};

} // namespace NAV
