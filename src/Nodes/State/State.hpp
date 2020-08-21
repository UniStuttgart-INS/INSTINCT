/// @file State.hpp
/// @brief State Information Node
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/State/StateData.hpp"

namespace NAV
{
class State : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the object
    /// @param[in] options Program options string map
    State(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    State() = default;
    /// @brief Destructor
    ~State() override = default;
    /// @brief Copy constructor
    State(const State&) = delete;
    /// @brief Move constructor
    State(State&&) = delete;
    /// @brief Copy assignment operator
    State& operator=(const State&) = delete;
    /// @brief Move assignment operator
    State& operator=(State&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("State");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const override
    {
        return std::string_view("State");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        return {};
    }

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] constexpr NodeContext context() const override
    {
        return NodeContext::ALL;
    }

    /// @brief Returns the number of Ports
    /// @param[in] portType Specifies the port type
    /// @return The number of ports
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const override
    {
        switch (portType)
        {
        case PortType::In:
            return 1U;
        case PortType::Out:
            return 1U;
        }

        return 0U;
    }

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const override
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return StateData().type();
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return StateData().type();
            }
            break;
        }

        return std::string_view("");
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) override
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<StateData>(data);
            updateState(obs);
        }
    }

    /// @brief Requests the node to send out its data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) override
    {
        if (portIndex == 0)
        {
            return currentState;
        }

        return nullptr;
    }

  private:
    /// @brief Update the current State
    /// @param[in] New State
    void updateState(std::shared_ptr<StateData>& state);

    /// The current vehicle state
    std::shared_ptr<StateData> currentState;
};

} // namespace NAV
