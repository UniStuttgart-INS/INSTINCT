/**
 * @file DataCallback.hpp
 * @brief Abstract class for Callback Functionality
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include "util/Common.hpp"

#include <functional>
#include <memory>
#include <vector>
#include <unordered_map>

#include "NodeData/NodeData.hpp"

namespace NAV
{
// Forward declaration
class Node;

/// Abstract class for Callback Functionality
class DataCallback
{
  public:
    /// Enables the callbacks
    bool callbacksEnabled = false;

    /**
     * @brief Adds the supplied callback at the end of the callback list
     * 
     * @param[in] port Port of the data callbacks
     * @param[in] callback Function pointer which should be called
     * @param[in] userData Pointer to user data that are needed when executing the callback
     * @retval NavStatus Indicates if adding the callback was successfull
     */
    NavStatus addCallback(size_t port, std::function<NavStatus(std::shared_ptr<NodeData>, std::shared_ptr<Node>)> callback, std::shared_ptr<Node> userData);

    /**
     * @brief Adds the supplied callback at the end of the callback list
     * 
     * @param[in] port Port of the data callbacks
     * @param[in] callback Function pointer which should be called
     * @param[in] userData Pointer to user data that are needed when executing the callback
     * @param[in] index Index where to insert the callback
     * @retval NavStatus Indicates if inserting the callback was successfull
     */
    NavStatus addCallback(size_t port, std::function<NavStatus(std::shared_ptr<NodeData>, std::shared_ptr<Node>)> callback, std::shared_ptr<Node> userData, size_t index);

    /**
     * @brief Removes the last callback from the list
     * 
     * @param[in] port Port of the data callbacks
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeCallback(size_t port);

    /**
     * @brief Removes the callback at the specified position from the list
     * 
     * @param[in] port Port of the data callbacks
     * @param[in] index Index where to remove the callback
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeCallback(size_t port, size_t index);

    /**
     * @brief Removes all callbacks from the list for the specified port
     * 
     * @param[in] port Port of the data callbacks
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeAllCallbacks(size_t port);

    /**
     * @brief Removes all callbacks from the list
     * 
     * @retval NavStatus Indicates if the removal was successfull
     */
    NavStatus removeAllCallbacks();

    /**
     * @brief Calls all registered callbacks
     * 
     * @attention Needs to be called by all synchronous and asynchronous message receivers
     * @param[in] port Port of the data callbacks
     * @param[in] data The received data
     * @retval NavStatus Indicates if there was a problem with one of the callbacks
     */
    NavStatus invokeCallbacks(size_t port, std::shared_ptr<NodeData> data);

  protected:
    /// Construct a new Data Callback object
    DataCallback();

    /// Deletes the object
    ~DataCallback();

  private:
    /// Data Structure for Callbacks
    struct Callback
    {
        std::function<NavStatus(std::shared_ptr<NodeData>, std::shared_ptr<Node>)> callback = nullptr;
        std::shared_ptr<Node> data = nullptr;
    };

    /**
     * @brief Callback lists for each port which are called if a message is ready to be sent
     * @note std::vector is used here, as it has the faster iteration performance.
     *       Inserting and removing is only done once at the start of the program.
     */
    std::unordered_map<size_t, std::vector<Callback>> _callbacks;
};

} // namespace NAV