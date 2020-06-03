/**
 * @file Node.hpp
 * @brief Abstract Node Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-18
 */

#pragma once

#include <string>
#include <string_view>
#include <memory>
#include <deque>

#include "DataCallback.hpp"
#include "NodeData/NodeData.hpp"

namespace NAV
{
/// Abstract Node Class
class Node : public DataCallback
{
  public:
    /// Port Type
    enum PortType
    {
        In, ///< Input Port
        Out ///< Output Port
    };

    /// Node Context
    enum NodeContext
    {
        REAL_TIME,
        POST_PROCESSING,
        ALL
    };

    /**
     * @brief Construct a new Node object
     * 
     * @param[in] name Name of the Node
     */
    explicit Node(std::string name);

    Node() = default;                      ///< Default constructor
    ~Node() override = default;            ///< Destructor
    Node(const Node&) = delete;            ///< Copy constructor
    Node(Node&&) = delete;                 ///< Move constructor
    Node& operator=(const Node&) = delete; ///< Copy assignment operator
    Node& operator=(Node&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] virtual constexpr std::string_view type() const = 0;

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] virtual constexpr std::string_view category() const = 0;

    enum ConfigOptions
    {
        CONFIG_BOOL,          ///< Boolean: Default
        CONFIG_INT,           ///< Integer: Min, Default, Max
        CONFIG_FLOAT,         ///< Float: Min, Default, Max
        CONFIG_STRING,        ///< String
        CONFIG_LIST,          ///< List: "option1|[default]|option3"
        CONFIG_LIST_LIST_INT, ///< 2 Lists and Integer: "[List1default]|List1option2||List2option1|[List2default]||min|default|max"
        CONFIG_MAP_INT,       ///< String Key and Integer Value: "key", "min", "default", "max"
    };

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> The gui configuration
     */
    [[nodiscard]] virtual std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> guiConfig() const = 0;

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] virtual constexpr NodeContext context() const = 0;

    /**
     * @brief Returns the number of Ports
     * 
     * @param[in] portType Specifies the port type
     * @retval constexpr uint8_t The number of ports
     */
    [[nodiscard]] virtual constexpr uint8_t nPorts(PortType portType) const = 0;

    /**
     * @brief Returns the data types provided by this class
     * 
     * @param[in] portType Specifies the port type
     * @param[in] portIndex Port index on which the data is sent
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] virtual constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const = 0;

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    virtual void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) = 0;

    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] virtual std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) = 0;

    /**
     * @brief Requests the node to peek its output data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] virtual std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t portIndex) = 0;

    /**
     * @brief Resets the node. In case of file readers, that moves the read cursor to the start
     */
    virtual void resetNode() {}

    /**
     * @brief Get the name string of the Node
     * 
     * @retval const std::string_view& The Name of the Node
     */
    [[nodiscard]] const std::string& getName() const;

  protected:
    /// Name of the Node
    const std::string name;
};

} // namespace NAV
