/**
 * @file DataCallback.hpp
 * @brief Abstract class for Callback Functionality
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <typeindex>

namespace NAV
{
// Forward declaration
class Node;
class NodeData;

/// Abstract class for Callback Functionality
class DataCallback
{
  public:
    /// Enables the callbacks
    bool callbacksEnabled = false;

    /**
     * @brief Adds the supplied node at the end of the callback list
     * 
     * @tparam T Output Message Class
     * @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Ensures template only exists for classes with base class 'NodeData'
     * @param[in] node Pointer to the node which should receive the callback
     * @param[in] portIndex Port of the data callbacks
     */
    template<class T,
             typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
    void addCallback(std::shared_ptr<Node>& node, uint8_t portIndex)
    {
        callbackList<T>().emplace_back(std::make_pair(node, portIndex));
    }

    /**
     * @brief Removes all callbacks from the list
     * 
     * @tparam T Output Message Class
     * @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Ensures template only exists for classes with base class 'NodeData'
     */
    template<class T,
             typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
    void removeAllCallbacks()
    {
        // Clear does not erase the memory the pointers point at
        // This is not a problem however, as we use shared_ptr
        for (auto& [node, portIndex] : callbackList<T>())
        {
            node = nullptr;
        }

        callbackList<T>().clear();

        callbacksEnabled = false;
    }

    /**
     * @brief Calls all registered callbacks
     * 
     * @attention Needs to be called by all synchronous and asynchronous message receivers
     * 
     * @tparam T Output Message Class
     * @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Ensures template only exists for classes with base class 'NodeData'
     * @param[in] data The data to pass to the callback targets
     */
    template<class T,
             typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
    void invokeCallbacks(const std::shared_ptr<T>& data)
    {
        if (callbacksEnabled)
        {
            for (const auto& [node, portIndex] : callbackList<T>())
            {
                node->handleInputData(portIndex, data);
            }
        }
    }

    /// List of links which are connected to this node
    /// Map key: portIndex of this node where the data are coming in
    /// Map val: Source Node which sends the data and its portIndex
    std::map<uint8_t, std::pair<std::shared_ptr<Node>, uint8_t>> incomingLinks;

    DataCallback(const DataCallback&) = delete;            ///< Copy constructor
    DataCallback(DataCallback&&) = delete;                 ///< Move constructor
    DataCallback& operator=(const DataCallback&) = delete; ///< Copy assignment operator
    DataCallback& operator=(DataCallback&&) = delete;      ///< Move assignment operator

  protected:
    /// Construct a new Data Callback object
    DataCallback() = default;

    /// Deletes the object
    virtual ~DataCallback() = default;

  private:
    /**
     * @brief Returns the callback list for the specified Message type
     * 
     * @tparam T Output Message class
     * @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Ensures template only exists for classes with base class 'NodeData'
     * @retval std::vector<std::pair<std::shared_ptr<Node>, uint8_t>>& Nodes and ports to call upon when invoking a callback
     * 
     * @note std::vector is used here, as it has the faster iteration performance.
     *       Inserting and removing is only done once at the start of the program.
     */
    template<class T,
             typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
    std::vector<std::pair<std::shared_ptr<Node>, uint8_t>>& callbackList()
    {
        return _callbackList[typeid(T)];
    }

    /// Internal callback list representation
    std::map<std::type_index, std::vector<std::pair<std::shared_ptr<Node>, uint8_t>>> _callbackList;
};

} // namespace NAV