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

    /// Default destructor
    virtual ~FileReader();

    /// Path to log file
    std::string path;
    /// File stream to read the file
    std::ifstream filestream;
};

} // namespace NAV
