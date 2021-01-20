/// @file NodeRegistry.hpp
/// @brief Utility class which specifies available nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-20

#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>

namespace NAV
{
class Node;

namespace NodeRegistry
{
struct NodeInfo
{
    /// Constructor
    std::function<Node*()> constructor;
    /// Class Type of the node
    std::string type;
    /// Category of the node
    std::string category;
};

/// @brief Reference to List of all registered Nodes
const std::vector<NodeInfo>& registeredNodes();

bool NodeDataTypeIsChildOf(const std::vector<std::string>& childTypes, const std::vector<std::string>& parentTypes);

/// @brief Register all available Node types for the program
void registerNodeTypes();

/// @brief Register all available NodeData types for the program
void registerNodeDataTypes();

} // namespace NodeRegistry

} // namespace NAV