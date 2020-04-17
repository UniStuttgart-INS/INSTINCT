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
