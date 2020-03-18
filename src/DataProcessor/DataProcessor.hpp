/**
 * @file DataProcessor.hpp
 * @brief Abstract Data Processor Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include <string>

#include "util/Common.hpp"
#include "DataCallback.hpp"

namespace NAV
{
/// Abstract Data Processor Class
class DataProcessor : public DataCallback
{
  public:
    /**
     * @brief  Initialize the Data Processor
     * 
     * @retval NavStatus Indicates whether initialization was successful
     */
    virtual NavStatus initialize() = 0;

    /**
     * @brief  Deinitialize the Data Processor
     * 
     * @retval NavStatus Indicates whether deinitialization was successful
     */
    virtual NavStatus deinitialize() = 0;

    /**
     * @brief Checks if the Data Processor is initialized
     * 
     * @retval bool Initialization state of the Data Processor
     */
    bool isInitialized();

    /**
     * @brief Get the name string of the Data Provider
     * 
     * @retval std::string The Name of the Data Provider
     */
    std::string getName();

  protected:
    /**
     * @brief Construct a new Data Processor object
     * 
     * @param[in] name Name of the Data Processor
     */
    DataProcessor(std::string name);

    /// Destroy the Data Processor object
    ~DataProcessor();

    /// Name of the Data Provider. For Sensors this equals the Model Number
    std::string name;

    /// Initialized flag
    bool initialized = false;
};

} // namespace NAV
