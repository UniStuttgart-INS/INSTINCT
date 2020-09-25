/// @file ImuSimulator.hpp
/// @brief Imu Observation Simulator
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "../Imu.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/StateData.hpp"

namespace NAV
{
/// File Reader for Imu log files
class ImuSimulator final : public Imu
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Sensor which wrote the file
    /// @param[in] options Program options string map
    ImuSimulator(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    ImuSimulator() = default;
    /// @brief Destructor
    ~ImuSimulator() final = default;
    /// @brief Copy constructor
    ImuSimulator(const ImuSimulator&) = delete;
    /// @brief Move constructor
    ImuSimulator(ImuSimulator&&) = delete;
    /// @brief Copy assignment operator
    ImuSimulator& operator=(const ImuSimulator&) = delete;
    /// @brief Move assignment operator
    ImuSimulator& operator=(ImuSimulator&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("ImuSimulator");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataSimulator");
    }

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::POST_PROCESSING;
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
            return 2U;
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
                return StateData().type();
            }
        case PortType::Out:
            if (portIndex == 0)
            {
                return ImuObs().type();
            }
            if (portIndex == 1)
            {
                return ImuPos().type();
            }
        }

        return std::string_view("");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        std::vector<ConfigOptions> configs = { { CONFIG_STRING, "Duration", "Duration of the data gerneation in [s]", { "1" } },
                                               { CONFIG_STRING, "Frequency", "Frequency of the data generation in [Hz]", { "10" } } };
        auto imuConfigs = Imu::guiConfig();
        configs.insert(configs.end(), imuConfigs.begin(), imuConfigs.end());
        return configs;
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData([[maybe_unused]] uint8_t portIndex, [[maybe_unused]] std::shared_ptr<NodeData> data) final {}

    /// @brief Requests the node to send out its data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) final
    {
        if (portIndex == 0)
        {
            return pollData();
        }
        if (portIndex == 1)
        {
            return imuPos;
        }

        return nullptr;
    }

    /// @brief Requests the node to peek its output data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t portIndex) final
    {
        if (portIndex == 0)
        {
            return pollData(true);
        }

        return nullptr;
    }

  private:
    /// @brief Polls the next simulated data
    /// @param[in] peek Specifies if the data should be peeked or read
    /// @return The simulated observation
    [[nodiscard]] std::shared_ptr<ImuObs> pollData(bool peek = false);

    /// Duration of the data gerneation in [s]
    double duration = 1.0;
    /// Frequency of the data generation in [Hz]
    double frequency = 10.0;
    /// Current Simulation Time in [s]
    double currentSimTime = 0.0;
};

} // namespace NAV
