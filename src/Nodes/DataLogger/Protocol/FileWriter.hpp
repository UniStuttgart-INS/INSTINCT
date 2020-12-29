/// @file FileWriter.hpp
/// @brief File Writer class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace NAV
{
class FileWriter
{
  public:
    /// File Type
    enum FileType
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        ASCII,  ///< Ascii text data
    };

    /// @brief Copy constructor
    FileWriter(const FileWriter&) = delete;
    /// @brief Move constructor
    FileWriter(FileWriter&&) = delete;
    /// @brief Copy assignment operator
    FileWriter& operator=(const FileWriter&) = delete;
    /// @brief Move assignment operator
    FileWriter& operator=(FileWriter&&) = delete;

  protected:
    /// @brief Default constructor
    FileWriter() = default;
    /// @brief Destructor
    ~FileWriter() = default;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j);

    /// @brief Initialize the file reader
    void initialize();

    /// @brief Deinitialize the file reader
    void deinitialize();

    /// Path to the file
    std::string path;

    /// File stream to write the file
    std::ofstream filestream;

    /// File Type
    FileType fileType = FileType::NONE;
};

} // namespace NAV
