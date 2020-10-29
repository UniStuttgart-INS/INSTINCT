/// @file ImuDataLogger.hpp
/// @brief Data Logger for Imu observations
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-06-30

#pragma once

#include "../DataLogger.hpp"

#include "NodeData/IMU/ImuObs.hpp"

namespace NAV
{
/// Data Logger for IMU observations
class ImuDataLogger final : public DataLogger
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Logger
    /// @param[in] options Program options string map
    ImuDataLogger(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    ImuDataLogger() = default;
    /// @brief Destructor
    ~ImuDataLogger() final;
    /// @brief Copy constructor
    ImuDataLogger(const ImuDataLogger&) = delete;
    /// @brief Move constructor
    ImuDataLogger(ImuDataLogger&&) = delete;
    /// @brief Copy assignment operator
    ImuDataLogger& operator=(const ImuDataLogger&) = delete;
    /// @brief Move assignment operator
    ImuDataLogger& operator=(ImuDataLogger&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("ImuDataLogger");
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
        return { { CONFIG_STRING, "Path", "Path where to save the data to", { "logs/imu-log.csv" } },
                 { CONFIG_LIST, "Type", "Type of the output file", { "[ascii]" } } };
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
                return std::make_pair(ImuObs().type(), std::string_view(""));
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
            auto obs = std::static_pointer_cast<ImuObs>(data);
            writeObservation(obs);
        }
    }

  private:
    /// @brief Write Observation to the file
    /// @param[in] obs The received observation
    void writeObservation(std::shared_ptr<ImuObs>& obs);
};

} // namespace NAV