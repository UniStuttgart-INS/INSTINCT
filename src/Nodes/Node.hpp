/**
 * @file Node.hpp
 * @brief Abstract Node Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include <string>
#include <deque>

#include "util/Common.hpp"
#include "DataCallback.hpp"

namespace NAV
{
/// Abstract Node Class
class Node : public DataCallback
{
  public:
    /**
     * @brief Get the name string of the Node
     * 
     * @retval std::string The Name of the Node
     */
    std::string getName();

    /**
     * @brief Checks if the Node is a File Reader Type and can provide data packages
     * 
     * @retval bool Indicates whether the class is a File Reader
     */
    virtual bool isFileReader();

    /**
     * @brief Poll the node to send out its data. Only implemented by file readers. Otherwise returns nullptr
     * 
     * @attention Overrides of this function have to invoke the Callback handlers
     * @retval std::shared_ptr<NodeData> The polled observation
     */
    virtual std::shared_ptr<NodeData> pollData();

    /**
     * @brief Peeks the next event time. Only implemented by file readers. Otherwise returns std::nullopt
     * 
     * @retval std::optional<uint64_t> Next event timestamp
     */
    virtual std::optional<uint64_t> peekNextUpdateTime();

  protected:
    /**
     * @brief Construct a new Node object
     * 
     * @param[in] name Name of the Node
     */
    Node(const std::string name);

    /// Destroy the Node object
    ~Node();

    /// Name of the Node
    const std::string name;
};

} // namespace NAV
