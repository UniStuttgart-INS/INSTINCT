/**
 * @file Node.hpp
 * @brief Abstract Node Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include <string>

#include "util/Common.hpp"
#include "DataCallback.hpp"

namespace NAV
{
/// Abstract Node Class
class Node : public DataCallback
{
  public:
    /**
     * @brief  Initialize the Node
     * 
     * @retval NavStatus Indicates whether initialization was successful
     */
    virtual NavStatus initialize() = 0;

    /**
     * @brief  Deinitialize the Node
     * 
     * @retval NavStatus Indicates whether deinitialization was successful
     */
    virtual NavStatus deinitialize() = 0;

    /**
     * @brief Checks if the Node is initialized
     * 
     * @retval bool Initialization state of the Node
     */
    bool isInitialized();

    /**
     * @brief Get the name string of the Node
     * 
     * @retval std::string The Name of the Node
     */
    std::string getName();

  protected:
    /**
     * @brief Construct a new Node object
     * 
     * @param[in] name Name of the Node
     */
    Node(std::string name);

    /// Destroy the Node object
    ~Node();

    /// Name of the Node
    std::string name;

    /// Initialized flag
    bool initialized = false;
};

} // namespace NAV
