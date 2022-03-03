/// @file FileWriter.hpp
/// @brief File Writer class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
/// @brief Parent class for other data loggers which manages the output filestream
class FileWriter
{
  public:
    /// File Type
    enum class FileType
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        CSV,    ///< Ascii text data
    };

    /// @brief Converts the provided type into string
    /// @param[in] type FileType to convert
    /// @return String representation of the type
    static const char* to_string(FileType type);

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
    bool initialize();

    /// @brief Deinitialize the file reader
    void deinitialize();

    /// Path to the file
    std::string _path;

    /// File stream to write the file
    std::ofstream _filestream;

    /// File Type
    FileType _fileType = FileType::NONE;
};

} // namespace NAV
