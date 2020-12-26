/// @file EmlidDataLogger.hpp
/// @brief Data Logger for Emlid observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "../DataLogger.hpp"

#include "NodeData/GNSS/EmlidObs.hpp"

namespace NAV
{
/// Data Logger for Emlid observations
class EmlidDataLogger final : public DataLogger
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Logger
    /// @param[in] options Program options string map
    EmlidDataLogger(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    EmlidDataLogger() = default;
    /// @brief Destructor
    ~EmlidDataLogger() final;
    /// @brief Copy constructor
    EmlidDataLogger(const EmlidDataLogger&) = delete;
    /// @brief Move constructor
    EmlidDataLogger(EmlidDataLogger&&) = delete;
    /// @brief Copy assignment operator
    EmlidDataLogger& operator=(const EmlidDataLogger&) = delete;
    /// @brief Move assignment operator
    EmlidDataLogger& operator=(EmlidDataLogger&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("EmlidDataLogger");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataLogger");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        return { { CONFIG_STRING, "Path", "Path where to save the data to", { "logs/Er-log.ubx" } },
                 { CONFIG_LIST, "Type", "Type of the output file", { "[binary]" } } };
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
            break;
        }

        return 0U;
    }

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type and subtitle
    [[nodiscard]] constexpr std::pair<std::string_view, std::string_view> dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return std::make_pair(EmlidObs::type(), std::string_view(""));
            }
        case PortType::Out:
            break;
        }

        return std::make_pair(std::string_view(""), std::string_view(""));
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<EmlidObs>(data);
            writeObservation(obs);
        }
    }

  private:
    /// @brief Write Observation to the file
    /// @param[in] obs The received observation
    void writeObservation(std::shared_ptr<EmlidObs>& obs);
};

} // namespace NAV
