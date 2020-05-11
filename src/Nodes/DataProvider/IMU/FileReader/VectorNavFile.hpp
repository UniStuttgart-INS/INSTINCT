/**
 * @file VectorNavFile.hpp
 * @brief File Reader for Vector Nav log files
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-16
 */

#pragma once

#include "../Imu.hpp"
#include "../../Protocol/FileReader.hpp"
#include "vn/sensors.h"

namespace NAV
{
/// File Reader for Vector Nav log files
class VectorNavFile final : public FileReader, public Imu
{
  public:
    /// Config Structure for the sensor
    using Config = struct
    {
    };

    /**
     * @brief Construct a new Vector Nav File object
     * 
     * @param[in] name Name of the Sensor which wrote the file
     * @param[in, out] options Program options string list
     */
    VectorNavFile(const std::string& name, std::deque<std::string>& options);

    /// Default destructor
    ~VectorNavFile() final;

    VectorNavFile(const VectorNavFile&) = delete;            ///< Copy constructor
    VectorNavFile(VectorNavFile&&) = delete;                 ///< Move constructor
    VectorNavFile& operator=(const VectorNavFile&) = delete; ///< Copy assignment operator
    VectorNavFile& operator=(VectorNavFile&&) = delete;      ///< Move assignment operator

    /**
     * @brief Checks if the Node is a File Reader Type and can provide data packages
     * 
     * @retval bool Indicates whether the class is a File Reader
     */
    bool isFileReader() final;

    /**
     * @brief Polls the next data package
     * 
     * @retval std::shared_ptr<NodeData> Pointer to an ImuObs object with the next data
     */
    std::shared_ptr<NodeData> pollData() final;

    /**
     * @brief Peeks the next line in the file for the time without moving the read cursor
     * 
     * @retval std::optional<uint64_t> Next timestamp in the log file
     */
    std::optional<InsTime> peekNextUpdateTime() final;

  private:
    /**
     * @brief Determines the type of the file (ASCII or binary)
     * 
     * @retval NavStatus Indicates whether the file type could be determined
     */
    NavStatus determineFileType();

    /// Whether the file is a binary file or ASCII file
    bool isBinary;

    /// Header Columns of an ASCII file
    std::vector<std::string> columns;

    /// Config Object
    VectorNavFile::Config config;
};

} // namespace NAV
