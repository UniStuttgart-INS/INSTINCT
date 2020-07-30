/// @file StateEstimator.hpp
/// @brief State Estimator Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/InsObs.hpp"

#include "util/Logger.hpp"

namespace NAV
{
class StateEstimator : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the object
    /// @param[in] options Program options string map
    StateEstimator(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    StateEstimator() = default;
    /// @brief Destructor
    ~StateEstimator() override = default;
    /// @brief Copy constructor
    StateEstimator(const StateEstimator&) = delete;
    /// @brief Move constructor
    StateEstimator(StateEstimator&&) = delete;
    /// @brief Copy assignment operator
    StateEstimator& operator=(const StateEstimator&) = delete;
    /// @brief Move assignment operator
    StateEstimator& operator=(StateEstimator&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("StateEstimator");
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
        return {
            { CONFIG_LIST, "Filter Type", "Select the type of the filter which should be used", { "KF", "[EKF]", "EnKF", "UKF" } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_STRING, "KF-Option1", "KF-Option1 ToolTip", { "" }), ConfigOptionsBase(CONFIG_BOOL, "EKF-Option1", "EKF-Option1 Tooltip1", { "1" }), ConfigOptionsBase(CONFIG_BOOL, "EnKF-Option1", "EnKF-Option1 Tooltip1", { "1" }), ConfigOptionsBase(CONFIG_BOOL, "UKF-Option1", "UKF-Option1 Tooltip1", { "1" }) } },
            { CONFIG_N_INPUT_PORTS, "Input Ports", "Amount of Input Ports", { "2", "2", "30", "4" } },
            { CONFIG_LIST, "Port Type", "Select the type of the message to receive on this port", { "VectorNavObs", "[UbloxObs]" } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_STRING, "VectorNavOption1", "VectorNavOption1 ToolTip", { "" }), ConfigOptionsBase(CONFIG_BOOL, "UbloxOption1", "UbloxOption1 Tooltip1", { "1" }) } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_BOOL, "VectorNavOption2", "VectorNavOption2 ToolTip", { "0" }), ConfigOptionsBase(CONFIG_LIST, "UbloxOption2", "UbloxOption1 Tooltip2", { "1", "[2]" }) } },
            { CONFIG_BOOL, "Bool2", "Use the Time configured here as start time", { "1" } },
        };
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
            return nInputPorts;
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
                return InsObs().type();
            }
            break;

        case PortType::Out:
            if (portIndex == 0)
            {
                return InsObs().type();
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
        static bool once = true;
        if (once)
        {
            LOG_INFO("Data on Port {}", portIndex);
            once = false;
        }
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<InsObs>(data);
            processObservation(obs);
        }
    }

  private:
    /// @brief Process the Observation data and invoke callbacks
    /// @param[in] obs Observation to process
    void processObservation(std::shared_ptr<InsObs>& obs);

    /// Number of input ports
    uint8_t nInputPorts = 1;
};

} // namespace NAV
