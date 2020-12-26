/// @file ImuIntegrator.hpp
/// @brief Integrates ImuObs Data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-18

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/StateData.hpp"

#include "Nodes/State/State.hpp"

namespace NAV
{
class ImuIntegrator : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the object
    /// @param[in] options Program options string map
    ImuIntegrator(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    ImuIntegrator() = default;
    /// @brief Destructor
    ~ImuIntegrator() override = default;
    /// @brief Copy constructor
    ImuIntegrator(const ImuIntegrator&) = delete;
    /// @brief Move constructor
    ImuIntegrator(ImuIntegrator&&) = delete;
    /// @brief Copy assignment operator
    ImuIntegrator& operator=(const ImuIntegrator&) = delete;
    /// @brief Move assignment operator
    ImuIntegrator& operator=(ImuIntegrator&&) = delete;

    /// @brief Initialize the Node. Here virtual functions of children can be called
    void initialize() final;

    /// @brief Initialize the Node. Here virtual functions of children can be called
    void deinitialize() final;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("ImuIntegrator");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const override
    {
        return std::string_view("Integrator");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const override
    {
        return { { CONFIG_LIST, "Integration Frame", "", { "[ECEF]", "NED" } } };
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
            return 2U;
        case PortType::Out:
            return 1U;
        }

        return 0U;
    }

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type and subtitle
    [[nodiscard]] constexpr std::pair<std::string_view, std::string_view> dataType(PortType portType, uint8_t portIndex) const override
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return std::make_pair(ImuObs::type(), std::string_view(""));
            }
            if (portIndex == 1)
            {
                return std::make_pair(StateData::type(), std::string_view(""));
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return std::make_pair(StateData::type(), std::string_view(""));
            }
            break;
        }

        return std::make_pair(std::string_view(""), std::string_view(""));
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) override
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<ImuObs>(data);
            integrateObservation(obs);
        }
    }

  private:
    /// @brief Integrates the Imu Observation data
    /// @param[in] imuObs__t0 ImuObs to process
    void integrateObservation(std::shared_ptr<ImuObs>& imuObs__t0);

    /// IMU Observation at the time tₖ₋₁
    std::shared_ptr<NAV::ImuObs> imuObs__t1 = nullptr;
    /// IMU Observation at the time tₖ₋₂
    std::shared_ptr<NAV::ImuObs> imuObs__t2 = nullptr;

    /// State Data at the time tₖ₋₂
    std::shared_ptr<StateData> stateData__t2 = nullptr;

    /// Pointer to the State Node connected on the StateData port
    std::shared_ptr<State> stateNode = nullptr;
    /// Output Port Index of the state node to request the StateData
    uint8_t stateNodeOutputPortIndex = 200;

    enum IntegrationFrame
    {
        ECEF,
        NED
    };
    IntegrationFrame integrationFrame = IntegrationFrame::ECEF;
};

} // namespace NAV
