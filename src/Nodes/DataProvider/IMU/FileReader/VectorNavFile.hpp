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
class VectorNavFile : public Imu, public FileReader
{
  public:
    /// Config Structure for the sensor
    typedef struct Config
    {
    } Config;

    /**
     * @brief Construct a new Vector Nav File object
     * 
     * @param[in] name Name of the Sensor which wrote the file
     * @param[in] path Path to the file to read
     * @param[in] sensorConfig Config Structure for the VectorNavFile
     */
    VectorNavFile(std::string name, std::string path, const VectorNavFile::Config sensorConfig);

    /// Default destructor
    virtual ~VectorNavFile();

    /**
     * @brief Initialize the File
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize() final;

    /**
     * @brief Deinitialize the file
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize() final;

    /**
     * @brief Polls the next data package
     * 
     * @retval std::shared_ptr<InsObs> Pointer to an ImuObs object with the next data
     */
    std::shared_ptr<InsObs> pollObservation() final;

    /**
     * @brief Peeks the next line in the file for the time without moving the read cursor
     * 
     * @retval std::optional<uint64_t> Next timestamp in the log file
     */
    std::optional<uint64_t> peekNextUpdateTime() final;

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
    const VectorNavFile::Config config;
};

} // namespace NAV
