/// @file FileReader.hpp
/// @brief Abstract File Reader class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include "Navigation/Time/InsTime.hpp"

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

    /// @brief Destructor
    virtual ~FileReader() = default;
    /// @brief Copy constructor
    FileReader(const FileReader&) = delete;
    /// @brief Move constructor
    FileReader(FileReader&&) = delete;
    /// @brief Copy assignment operator
    FileReader& operator=(const FileReader&) = delete;
    /// @brief Move assignment operator
    FileReader& operator=(FileReader&&) = delete;

    /// Results enum for the gui config
    enum GuiResult
    {
        PATH_UNCHANGED = 0,   ///< No changes made
        PATH_CHANGED,         ///< The path changed and exists
        PATH_CHANGED_INVALID, ///< The path changed but does not exist or is invalid
    };

  protected:
    /// @brief Default constructor
    FileReader() = default;

    /// @brief ImGui config
    /// @param[in] vFilters Filter to apply for file names
    /// @param[in] extensions Extensions to filter
    /// @param[in] id Unique id for creating the dialog uid
    /// @param[in] nameId Name of the node triggering the window used for logging
    /// @return True if changes occurred
    GuiResult guiConfig(const char* vFilters, const std::vector<std::string>& extensions, size_t id, const std::string& nameId);

    /// @brief Returns the path of the file
    std::filesystem::path getFilepath();

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
    std::string _path;
    /// File stream to read the file
    std::ifstream _filestream;
    /// File Type
    FileType _fileType = FileType::NONE;
    /// Start of the data in the file
    std::streampos _dataStart = 0;

    /// Header Columns of a CSV file
    std::vector<std::string> _headerColumns;
};

} // namespace NAV
