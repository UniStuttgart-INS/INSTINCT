/// @file TimeLimiter.hpp
/// @brief Limits the messages to the specified time range
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-04-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/GNSS/EmlidObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Limits the messages to the specified time range
class TimeLimiter final : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Object
    /// @param[in] options Program options string map
    TimeLimiter(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    TimeLimiter() = default;
    /// @brief Destructor
    ~TimeLimiter() final = default;
    /// @brief Copy constructor
    TimeLimiter(const TimeLimiter&) = delete;
    /// @brief Move constructor
    TimeLimiter(TimeLimiter&&) = delete;
    /// @brief Copy assignment operator
    TimeLimiter& operator=(const TimeLimiter&) = delete;
    /// @brief Move assignment operator
    TimeLimiter& operator=(TimeLimiter&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("TimeLimiter");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("Time");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        return {
            { CONFIG_LIST, "1-Port Type", "Select the type of the message to receive on this port", { "[" + std::string(VectorNavObs().type()) + "]", std::string(ImuObs().type()), std::string(KvhObs().type()), std::string(EmlidObs().type()), std::string(RtklibPosObs().type()), std::string(UbloxObs().type()) } },
            { CONFIG_INT, "Lower Limit\nGps Cycle", "GPS Cycle of lower limit", { "0", "0", "10" } },
            { CONFIG_INT, "Lower Limit\nGps Week", "GPS Week of lower limit", { "0", "0", "245760" } },
            { CONFIG_FLOAT, "Lower Limit\nGps Time of Week", "GPS Time of Week of lower limit", { "0", "0", "604800", "3" } },
            { CONFIG_INT, "Upper Limit\nGps Cycle", "GPS Cycle of upper limit", { "0", "0", "10" } },
            { CONFIG_INT, "Upper Limit\nGps Week", "GPS Week of upper limit", { "0", "0", "245760" } },
            { CONFIG_FLOAT, "Upper Limit\nGps Time of Week", "GPS Time of Week of upper limit", { "0", "0", "604800", "3" } },
        };
    }

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::ALL;
    }

    /// @brief Returns the number of Ports
    /// @param[in] portType Specifies the port type
    /// @return The number of ports
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const final
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
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return portDataType;
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return portDataType;
            }
            break;
        }

        return std::string_view("");
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<InsObs>(data);

            if (obs->insTime < lowerLimit || obs->insTime > upperLimit)
            {
                return;
            }

            if (portDataType == VectorNavObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<VectorNavObs>(obs));
            }
            else if (portDataType == ImuObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<ImuObs>(obs));
            }
            else if (portDataType == KvhObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<KvhObs>(obs));
            }
            else if (portDataType == EmlidObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<EmlidObs>(obs));
            }
            else if (portDataType == RtklibPosObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<RtklibPosObs>(obs));
            }
            else if (portDataType == UbloxObs().type())
            {
                invokeCallbacks(std::static_pointer_cast<UbloxObs>(obs));
            }
        }
    }

  private:
    /// Lower Time limit
    InsTime lowerLimit;

    /// Upper Time limit
    InsTime upperLimit;

    /// Input and output Data Types
    std::string portDataType;
};

} // namespace NAV
