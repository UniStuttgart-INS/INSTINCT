/// @file NodeManager.hpp
/// @brief Manages all Nodes
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-05-12

#pragma once

#include <vector>
#include <string>
#include <type_traits>
#include <functional>
#include <memory>
#include <map>
#include <cstdint>

#include "Node.hpp"

namespace NAV
{
class NodeManager
{
  public:
    /// Info about registered Node types
    struct NodeInfo
    {
        /// Constructor to call for new Node generation. Parameters are (name, options)
        std::function<std::shared_ptr<Node>(const std::string&, const std::map<std::string, std::string>&)> constructor;
        /// Constructor to call to get Node Info
        std::function<std::shared_ptr<Node>()> constructorEmpty;
    };

    /// Info about registered Node data types
    struct NodeDataInfo
    {
        /// Function to call when adding a callback. Parameters are (sourceNode, targetNode, targetPortIndex)
        std::function<void(std::shared_ptr<Node>&, std::shared_ptr<Node>&, uint8_t)> addCallback;
        /// Parent node data types
        std::vector<std::string_view> parents;
    };

    /// @brief Registers a Node with the NodeManager
    /// @tparam T Node Class to register
    /// @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Nodes'
    template<typename T,
             typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
    void registerNodeType()
    {
        NodeInfo info;
        info.constructor = [](const std::string& name, const std::map<std::string, std::string>& options) { return std::make_shared<T>(name, options); };
        info.constructorEmpty = []() { return std::make_shared<T>(); };
        _registeredNodes[T().type()] = info;
    }

    /// @brief Register a NodeData with the NodeManager
    /// @tparam T NodeData Class to register
    /// @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Makes sure template only exists for classes with base class 'NodeData'
    template<typename T,
             typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
    void registerNodeDataType()
    {
        NodeDataInfo info;
        info.addCallback = [](std::shared_ptr<Node>& sourceNode, std::shared_ptr<Node>& targetNode, uint8_t portIndex) { return sourceNode->addCallback<T>(targetNode, portIndex); };
        info.parents = T().parentTypes();
        _registeredNodeDataTypes[T().type()] = info;
    }

    /// @brief Processes the Config file nodes
    void processConfigFile();

    /// @brief Initialize all Nodes from the config file
    void initializeNodes();

    /// @brief Link all Nodes from the config file
    void linkNodes();

    /// @brief Enables callbacks for all Nodes from the config file
    void enableAllCallbacks();

    /// @brief Disables callbacks for all Nodes from the config file
    void disableAllCallbacks();

    /// @brief Deletes all nodes and frees memory except for the specified type
    /// @param[in] type The type which should not be deleted
    void deleteAllNodesExcept(const std::string_view& type);

    /// @brief Deletes all nodes and frees memory
    void deleteAllNodes();

    /// @brief Returns a list of all configured nodes
    /// @return All configured nodes
    [[nodiscard]] const std::vector<std::shared_ptr<Node>>& nodes() const;

    /// @brief Returns a list of all registered node types
    /// @return All registered node types
    [[nodiscard]] const std::map<std::string_view, NodeInfo>& registeredNodeTypes() const;

    /// @brief Returns a list of all registered node data types
    /// @return All registered node data types
    [[nodiscard]] const std::map<std::string_view, NodeDataInfo>& registeredNodeDataTypes() const;

    /// Specifies if the Application runs in real-time or post processing mode
    static Node::NodeContext appContext;

  private:
    /// Stores info to construct a node
    struct NodeConfig
    {
        /// Name of the Node
        std::string name;
        /// Type of the Node
        std::string type;
        /// Constructor options
        std::map<std::string, std::string> options;
    };

    /// Stores info to set up a data link
    struct NodeLink
    {
        /// Name of the Source Node
        std::string source;
        /// Port Index of the Source Node
        uint8_t sourcePortIndex = UINT8_MAX;
        /// Name of the Target Node
        std::string target;
        /// Port Index of the Target Node
        uint8_t targetPortIndex = UINT8_MAX;
    };

    /// List of all registered nodes
    std::map<std::string_view, NodeInfo> _registeredNodes;

    /// List of all registered node data types
    std::map<std::string_view, NodeDataInfo> _registeredNodeDataTypes;

    /// List of all constructed nodes
    std::vector<std::shared_ptr<Node>> _nodes;

    /// List of all Node Configs
    std::vector<NodeConfig> nodeConfigs;

    /// List of all Node Links
    std::vector<NodeLink> nodeLinks;
};

} // namespace NAV
