// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

#include <fmt/ostream.h>
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
        ASCII,  ///< Ascii text data
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
    ///
    /// The base implementation reads a CSV file header
    /// @attention If your file does not have a header, this functions needs to be overridden with an empty function
    virtual void readHeader();

    /// @brief Reads a line from the filestream
    /// @param str String to read the line into
    auto& getline(std::string& str)
    {
        _lineCnt++;
        return std::getline(_filestream, str);
    }

    /// @brief Extracts up to count immediately available characters from the input stream. The extracted characters are stored into the character array pointed to by s.
    /// @param[out] s pointer to the character array to store the characters to
    /// @param[in] count maximum number of characters to read
    /// @return The number of characters actually extracted.
    auto readsome(char* s, std::streamsize count) { return _filestream.readsome(s, count); }

    /// @brief Extraction without delimiters.
    /// @param __s A character array.
    /// @param __n Maximum number of characters to store.
    /// @return The filestream if the stream state is good(), extracts characters and stores them into __s until one of the following happens: - __n characters are stored - the input sequence reaches end-of-file, in which case the error state is set to failbit|eofbit.
    /// @note This function is not overloaded on signed char and unsigned char.
    auto& read(char* __s, std::streamsize __n) { return _filestream.read(__s, __n); }

    /// @brief Extracts and discards characters from the input stream until and including delim.
    /// @param count number of characters to extract
    /// @param delim delimiting character to stop the extraction at. It is also extracted.
    /// @return The filestream
    auto& ignore(std::streamsize count, int delim) { return _filestream.ignore(count, delim); }

    /// @brief Changing the current read position.
    /// @param pos A file offset object.
    /// @param dir The direction in which to seek.
    /// @return The filestream if fail() is not true, calls rdbuf()->pubseekoff(__off,__dir). If that function fails, sets failbit.
    /// @note This function first clears eofbit. It does not count the number of characters extracted, if any, and therefore does not affect the next call to gcount().
    auto& seekg(std::streamoff pos, std::ios_base::seekdir dir) { return _filestream.seekg(pos, dir); }

    /// @brief Getting the current read position.
    /// @return A file position object. If fail() is not false, returns pos_type(-1) to indicate failure. Otherwise returns rdbuf()->pubseekoff(0,cur,in).
    /// @note This function does not count the number of characters extracted, if any, and therefore does not affect the next call to gcount(). At variance with putback, unget and seekg, eofbit is not cleared first.
    [[nodiscard]] std::streampos tellg() { return _filestream.tellg(); }

    /// Check whether the end of file is reached
    [[nodiscard]] auto eof() const { return _filestream.eof(); }

    /// @brief Fast error checking.
    /// @return True if no error flags are set. A wrapper around rdstate.
    [[nodiscard]] bool good() const { return _filestream.good(); }

    /// @brief Looking ahead in the stream
    /// @return The next character, or eof(). If, after constructing the sentry object, good() is false, returns traits::eof(). Otherwise reads but does not extract the next input character.
    [[nodiscard]] auto peek() { return _filestream.peek(); }

    /// Get the current line number
    [[nodiscard]] size_t getCurrentLineNumber() const { return _lineCnt; }

    /// Path to the file
    std::string _path;
    /// File Type
    FileType _fileType = FileType::NONE;

    /// Header Columns of a CSV file
    std::vector<std::string> _headerColumns;

  private:
    /// File stream to read the file
    std::ifstream _filestream;
    /// Start of the data in the file
    std::streampos _dataStart = 0;
    /// Line counter
    size_t _lineCnt = 0;
    /// Line counter data start
    size_t _lineCntDataStart = 0;
};

} // namespace NAV

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::FileReader::FileType> : ostream_formatter
{};

#endif