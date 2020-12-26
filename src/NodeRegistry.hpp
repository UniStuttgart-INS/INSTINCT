/// @file NodeRegistry.hpp
/// @brief Utility class which specifies available nodes
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-14

#pragma once

namespace NAV
{
/// Forward declaration
class NodeManager;

/// Utility class which specifies available nodes
class NodeRegistry
{
  public:
    /// @brief Constructor
    NodeRegistry() = delete;

    /// @brief Register all available Node types for the program
    /// @param[in, out] nodeManager The NodeManager to register the Nodes with
    static void registerNodeTypes(NodeManager& nodeManager);

    /// @brief Register all available Node data types for the program
    /// @param[in, out] nodeManager The NodeManager to register the Nodes with
    static void registerNodeDataTypes(NAV::NodeManager& nodeManager);
};

} // namespace NAV
