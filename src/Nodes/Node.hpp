/// @file Node.hpp
/// @brief Abstract Node Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-18

#pragma once

#include <string>
#include <string_view>
#include <memory>
#include <map>
#include <variant>

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

    /// @brief Constructor
    /// @param[in] name Name of the Node
    explicit Node(std::string name);

    /// @brief Default constructor
    Node() = default;
    /// @brief Destructor
    ~Node() override = default;
    /// @brief Copy constructor
    Node(const Node&) = delete;
    /// @brief Move constructor
    Node(Node&&) = delete;
    /// @brief Copy assignment operator
    Node& operator=(const Node&) = delete;
    /// @brief Move assignment operator
    Node& operator=(Node&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] virtual constexpr std::string_view type() const = 0;

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] virtual constexpr std::string_view category() const = 0;

    /// @brief Config Option Types Enumeration
    enum ConfigOptionType
    {
        CONFIG_N_INPUT_PORTS,   ///< Integer: Min, Default, Max, Amount of Config Options to repeat
        CONFIG_BOOL,            ///< Boolean: Default
        CONFIG_INT,             ///< Integer: Min, Default, Max
        CONFIG_FLOAT,           ///< Float: Min, Default, Max
        CONFIG_STRING,          ///< String
        CONFIG_STRING_BOX,      ///< String Box
        CONFIG_LIST,            ///< List: "option1", "[default]", "option3"
        CONFIG_LIST_MULTI,      ///< List which repeats: "option1", "[default]", "option3"
        CONFIG_LIST_LIST_MULTI, ///< 2 Lists which repeat: "[List1default]", "List1option2", "|", "List2option1|[List2default]"
        CONFIG_MAP_INT,         ///< String Key and Integer Value: "key", "min", "default", "max"
        CONFIG_VARIANT,         ///< Variant: ConfigOptionsBase(option1), ConfigOptionsBase(option2)
    };

    using ConfigOptionsBase = std::tuple<ConfigOptionType, std::string, std::string, std::vector<std::string>>;
    using ConfigOptions = std::tuple<ConfigOptionType, std::string, std::string, std::vector<std::variant<std::string, ConfigOptionsBase>>>;

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] virtual std::vector<ConfigOptions> guiConfig() const = 0;

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] virtual constexpr NodeContext context() const = 0;

    /// @brief Returns the number of Ports
    /// @param[in] portType Specifies the port type
    /// @return The number of ports
    [[nodiscard]] virtual constexpr uint8_t nPorts(PortType portType) const = 0;

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type
    [[nodiscard]] virtual constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const = 0;

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    virtual void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) = 0;

    /// @brief Requests the node to send out its data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] virtual std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex);

    /// @brief Requests the node to peek its output data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] virtual std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t portIndex);

    /// @brief Resets the node. In case of file readers, that moves the read cursor to the start
    virtual void resetNode();

    /// @brief Get the name string of the Node
    /// @return The Name of the Node
    [[nodiscard]] const std::string& getName() const;

  protected:
    /// Name of the Node
    const std::string name;
};

} // namespace NAV
