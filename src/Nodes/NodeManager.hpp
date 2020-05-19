/**
 * @file NodeManager.hpp
 * @brief Manages all Nodes
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-05-12
 */

#pragma once

#include <vector>
#include <string>
#include <type_traits>
#include <functional>
#include <memory>
#include <map>
#include <deque>
#include <cstdint>

#include "Node.hpp"

namespace NAV
{
class NodeManager
{
  public:
    /// Info about registered Node types
    using NodeInfo = struct
    {
        /// Constructor to call for new Node generation. Parameters are (name, options)
        std::function<std::shared_ptr<Node>(const std::string&, std::deque<std::string>&)> constructor;
        /// Constructor to call to get Node Info
        std::function<std::shared_ptr<Node>()> constructorEmpty;
    };

    /// Info about registered Node data types
    using NodeDataInfo = struct
    {
        /// Function to call when adding a callback. Parameters are (sourceNode, targetNode, targetPortIndex)
        std::function<void(std::shared_ptr<Node>&, std::shared_ptr<Node>&, uint8_t)> addCallback;
        /// Parent node data types
        std::vector<std::string_view> parents;
    };

    /**
     * @brief Registers a Class with the NodeManager
     * 
     * @tparam T Class to register
     * @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Nodes'
     * @param[in] type Type of the Class
     */
    template<typename T,
             typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
    void registerNodeType()
    {
        NodeInfo info;
        info.constructor = [](const std::string& name, std::deque<std::string>& options) { return std::make_shared<T>(name, options); };
        info.constructorEmpty = []() { return std::make_shared<T>(); };
        _registeredNodes[T().type()] = info;
    }

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

    /// @brief Deletes all nodes and frees memory
    void deleteAllNodes();

    /**
     * @brief Returns a list of all configured nodes
     * 
     * @retval const std::vector<Node>& All configured nodes
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Node>>& nodes() const;

    /**
     * @brief Returns a list of all registered node types
     * 
     * @retval const std::map<std::string_view, NodeInfo>& All registered node types
     */
    [[nodiscard]] const std::map<std::string_view, NodeInfo>& registeredNodeTypes() const;

    /**
     * @brief Returns a list of all registered node data types
     * 
     * @retval const std::map<std::string_view, NodeInfo>& All registered node data types
     */
    [[nodiscard]] const std::map<std::string_view, NodeDataInfo>& registeredNodeDataTypes() const;

    /// Specifies if the Application runs in real-time or post processing mode
    static Node::NodeContext appContext;

  private:
    /// Stores info to construct a node
    using NodeConfig = struct
    {
        std::string name;                ///< Name of the Node
        std::string type;                ///< Type of the Node
        std::deque<std::string> options; ///< Constructor options
    };

    /// Stores info to set up a data link
    using NodeLink = struct
    {
        std::string source;                  ///< Name of the Source Node
        uint8_t sourcePortIndex = UINT8_MAX; ///< Port Index of the Source Node
        std::string target;                  ///< Name of the Target Node
        uint8_t targetPortIndex = UINT8_MAX; ///< Port Index of the Target Node
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
