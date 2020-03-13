/**
 * @file DataProvider.hpp
 * @brief Abstract Class for Data Providers
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include "util/Common.hpp"
#include "InsObs.hpp"

#include <memory>
#include <functional>
#include <vector>
#include <string>

namespace NAV
{
/// Abstract Class for Data Providers
class DataProvider
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

    /// Enables the callbacks
    bool callbacksEnabled = false;

    /**
     * @brief Adds the supplied callback at the end of the callback list
     * 
     * @param[in] callback Function pointer which should be called
     * @param[in] userData Pointer to user data that are needed when executing the callback
     * @retval NavStatus Indicates if adding the callback was successfull
     */
    NavStatus addObservationReceivedCallback(std::function<NavStatus(std::shared_ptr<InsObs>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData);

    /**
     * @brief Adds the supplied callback at the end of the callback list
     * 
     * @param[in] callback Function pointer which should be called
     * @param[in] userData Pointer to user data that are needed when executing the callback
     * @param[in] index Index where to insert the callback
     * @retval NavStatus Indicates if inserting the callback was successfull
     */
    NavStatus addObservationReceivedCallback(std::function<NavStatus(std::shared_ptr<InsObs>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData, size_t index);

    /**
     * @brief Removes the last callback from the list
     * 
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeObservationReceivedCallback();

    /**
     * @brief Removes the callback at the specified position from the list
     * 
     * @param[in] index Index where to remove the callback
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeObservationReceivedCallback(size_t index);

    /**
     * @brief Removes all callbacks from the list
     * 
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeAllObservationReceivedCallbacks();

  protected:
    /**
     * @brief Construct a new Data Provider object
     * 
     * @param[in] name Name of the Data Provider
     */
    DataProvider(const std::string name);

    /// Destroy the Data Provider object
    ~DataProvider();

    /**
     * @brief Calls all registered callbacks
     * 
     * @attention Needs to be called by all synchronous and asynchronous message receivers
     * @param[in] obs The received observation
     * @retval NavStatus Indicates if there was a problem with one of the callbacks
     */
    NavStatus observationReceived(std::shared_ptr<InsObs> obs);

    /// Name of the Data Provider. For Sensors this equals the Model Number
    std::string name;

    /// Initialized flag
    bool initialized = false;

  private:
    /**
     * @brief Callback list which is called if an observation is received
     * @note std::vector is used here, as it has the faster iteration performance.
     *       Inserting and removing is only done once at the start of the program.
     */
    std::vector<std::function<NavStatus(std::shared_ptr<InsObs>, std::shared_ptr<void>)>> observationReceivedCallbacks;

    /**
     * @brief Pointer list to user data, which is needed when observation is available
     * @note std::vector is used here, as it has the faster iteration performance.
     *       Inserting and removing is only done once at the start of the program.
     */
    std::vector<std::shared_ptr<void>> observationReceivedUserData;
};

} // namespace NAV
