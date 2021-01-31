/// @file NodeRegistry.hpp
/// @brief Utility class which specifies available nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-20

#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <map>

#include "internal/Pin.hpp"

namespace NAV
{
class Node;

namespace NodeRegistry
{
struct PinInfo
{
    PinInfo(const Pin::Kind& kind, const Pin::Type& type, std::vector<std::string> dataIdentifier)
        : kind(kind), type(type), dataIdentifier(std::move(dataIdentifier)) {}

    /// Kind of the Pin (Input/Output)
    Pin::Kind kind = Pin::Kind::None;
    /// Type of the Pin
    Pin::Type type = Pin::Type::None;
    /// One or multiple Data Identifiers (Unique name which is used for data flows)
    std::vector<std::string> dataIdentifier;
};

struct NodeInfo
{
    /// Constructor
    std::function<Node*()> constructor;
    /// Class Type of the node
    std::string type;
    /// List of port data types
    std::vector<PinInfo> pinInfoList;

    /// Class to Node address offset
    int64_t addressOffsetNode = 0;

    /// @brief Checks if the node has a pin which can be linked
    /// @param[in] pin Pin to link to
    [[nodiscard]] bool hasCompatiblePin(const Pin* pin) const;
};

/// @brief Reference to List of all registered Nodes
const std::map<std::string, std::vector<NodeInfo>>& RegisteredNodes();

bool NodeDataTypeIsChildOf(const std::vector<std::string>& childTypes, const std::vector<std::string>& parentTypes);

/// @brief Register all available Node types for the program
void RegisterNodeTypes();

/// @brief Register all available NodeData types for the program
void RegisterNodeDataTypes();

} // namespace NodeRegistry

} // namespace NAV