/**
 * @file DataProvider.hpp
 * @brief Abstract Class for Data Providers
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include "util/Common.hpp"
#include "InsObs.hpp"
#include "DataCallback.hpp"

#include <memory>
#include <vector>
#include <string>

namespace NAV
{
/// Abstract Class for Data Providers
class DataProvider : public DataCallback
{
  public:
    /**
     * @brief  Initialize the Data Provider
     * 
     * @retval NavStatus Indicates whether initialization was successful
     */
    virtual NavStatus initialize() = 0;

    /**
     * @brief  Deinitialize the Data Provider
     * 
     * @retval NavStatus Indicates whether deinitialization was successful
     */
    virtual NavStatus deinitialize() = 0;

    /**
     * @brief Poll the next/current observation
     * 
     * @attention Has to call the observationReceivedCallback handlers
     * @retval std::shared_ptr<InsObs> The polled observation
     */
    virtual std::shared_ptr<InsObs> pollObservation() = 0;

    /**
     * @brief Checks if the Data Provider is initialized
     * 
     * @retval bool Initialization state of the Data Provider
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
     * @brief Construct a new Data Provider object
     * 
     * @param[in] name Name of the Data Provider
     */
    DataProvider(const std::string name);

    /// Destroy the Data Provider object
    ~DataProvider();

    /// Name of the Data Provider. For Sensors this equals the Model Number
    std::string name;

    /// Initialized flag
    bool initialized = false;
};

} // namespace NAV
