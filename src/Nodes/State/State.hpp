/// @file State.hpp
/// @brief State Information Node
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/State/StateData.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuPos.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

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
        return {
            { CONFIG_BOOL, "Static\nInitialization", "Get the initial state from the GUI", { "0" } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_FLOAT, "Init Time", "Time in [s] to initialize the state.", { "0.01", "5", "60", "2" }), ConfigOptionsBase(CONFIG_FLOAT3, "Init LatLonAlt", "Initial values for Latitude [deg], Longitude [deg] and Height [m]", { "-89.99999999", "0", "89.99999999", "10", "-179.99999999", "0", "179.99999999", "10", "-1000", "0", "20000", "4" }) } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_EMPTY, "", "", {}), ConfigOptionsBase(CONFIG_FLOAT3, "Init RollPitchYaw", "Initial values for Roll, Pitch and Yaw in [deg]", { "-180", "0", "180", "2", "-90", "0", "90", "2", "-180", "0", "180", "2" }) } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_EMPTY, "", "", {}), ConfigOptionsBase(CONFIG_FLOAT3, "Init Velocity", "Initial velocity in navigation (NED) frame [m/s]", { "-100", "0", "100", "3", "-100", "0", "100", "3", "-100", "0", "100", "3" }) } },
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
            return 4U;
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
                return std::make_pair(StateData::type(), std::string_view("Update"));
            }
            if (portIndex == 1)
            {
                return std::make_pair(GnssObs::type(), std::string_view("Init"));
            }
            if (portIndex == 2)
            {
                return std::make_pair(ImuObs::type(), std::string_view("Init"));
            }
            if (portIndex == 3)
            {
                return std::make_pair(ImuPos::type(), std::string_view("Init"));
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
            auto obs = std::static_pointer_cast<StateData>(data);
            updateState(obs);
        }
        else if (portIndex == 1 && dynamicStateInit)
        {
            auto obs = std::static_pointer_cast<GnssObs>(data);
            initPositionVelocity(obs);
        }
        else if (portIndex == 2 && dynamicStateInit)
        {
            auto obs = std::static_pointer_cast<ImuObs>(data);
            initAttitude(obs);
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

    /// The initial vehicle state
    std::shared_ptr<StateData> initialState;

  private:
    /// @brief Initialize the State with Imu data
    /// @param[in] state Partial state data
    void initAttitude(std::shared_ptr<ImuObs>& obs);

    /// @brief Initialize the State with Gnss data
    /// @param[in] state Partial state data
    void initPositionVelocity(std::shared_ptr<GnssObs>& obs);

    /// @brief Finalize the initialization
    /// @param[in] currentTime The latest time
    void finalizeInit(const InsTime& currentTime);

    /// @brief Update the current State
    /// @param[in] state The new state
    void updateState(std::shared_ptr<StateData>& state);

    /// The current vehicle state
    std::shared_ptr<StateData> currentState;

    bool dynamicStateInit = false;

    double averageOverSeconds = 5;
    InsTime averageStartTime;

    double countAveragedAttitude = 0;
    double countAveragedPosition = 0;
    double countAveragedVelocity = 0;
};

} // namespace NAV
