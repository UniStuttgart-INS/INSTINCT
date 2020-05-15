/**
 * @file NodeRegistry.hpp
 * @brief Utility class which specifies available nodes
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-14
 */

#pragma once

namespace NAV
{
// Forward declaration
class NodeManager;

/// Utility class which specifies available nodes
class NodeRegistry
{
  public:
    // Constructor
    NodeRegistry() = delete;

    /// @brief Register all available Node types for the program
    static void registerNodeTypes(NodeManager& nodeManager);

    /// @brief Register all available Node data types for the program
    static void registerNodeDataTypes(NAV::NodeManager& nodeManager);
};

} // namespace NAV
