// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file FileWriter.hpp
/// @brief File Writer class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include <string>
#include <fstream>
#include <filesystem>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
/// @brief Parent class for other data loggers which manages the output filestream
class FileWriter
{
  public:
    /// File Type
    enum class FileType : uint8_t
    {
        NONE,   ///< Not specified
        BINARY, ///< Binary data
        ASCII,  ///< Ascii text data
    };

    /// @brief Converts the provided type into string
    /// @param[in] type FileType to convert
    /// @return String representation of the type
    static const char* to_string(FileType type);

    /// @brief Destructor
    ~FileWriter() = default;
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

    /// @brief ImGui config
    /// @param[in] vFilters Filter to apply for file names
    /// @param[in] extensions Extensions to filter
    /// @param[in] id Unique id for creating the dialog uid
    /// @param[in] nameId Name of the node triggering the window used for logging
    /// @return True if changes occurred
    bool guiConfig(const char* vFilters, const std::vector<std::string>& extensions, size_t id, const std::string& nameId);

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

    /// Path to the file
    std::string _path;

    /// File stream to write the file
    std::ofstream _filestream;

    /// File Type
    FileType _fileType = FileType::NONE;
};

} // namespace NAV
