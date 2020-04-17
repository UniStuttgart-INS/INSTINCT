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
    /**
     * @brief Peeks the next line in the file for the time without moving the read cursor
     * 
     * @retval std::optional<uint64_t> Next timestamp in the log file
     */
    virtual std::optional<uint64_t> peekNextUpdateTime() = 0;

  protected:
    /**
     * @brief Construct a new File Reader object
     * 
     * @param[in, out] options Program options string list
     */
    FileReader(std::deque<std::string>& options);

    /// Default destructor
    ~FileReader();

    /// Path to log file
    std::string path;
    /// File stream to read the file
    std::ifstream filestream;
};

} // namespace NAV
