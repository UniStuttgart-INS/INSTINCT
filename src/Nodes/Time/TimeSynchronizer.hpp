/// @file TimeSynchronizer.hpp
/// @brief Class to Synchronize Different Data Providers to the same Time base
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-21

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Class to Synchronize Different Data Providers to the same Time base
class TimeSynchronizer final : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Object
    /// @param[in] options Program options string map
    TimeSynchronizer(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    TimeSynchronizer() = default;
    /// @brief Destructor
    ~TimeSynchronizer() final = default;
    /// @brief Copy constructor
    TimeSynchronizer(const TimeSynchronizer&) = delete;
    /// @brief Move constructor
    TimeSynchronizer(TimeSynchronizer&&) = delete;
    /// @brief Copy assignment operator
    TimeSynchronizer& operator=(const TimeSynchronizer&) = delete;
    /// @brief Move assignment operator
    TimeSynchronizer& operator=(TimeSynchronizer&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("TimeSynchronizer");
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
            // clang-format off
            { CONFIG_LIST, "1-Port Type",
                           "Select the type of the message to receive on this port",
                           { "[" + std::string(VectorNavObs::type()) + "]", "",
                           std::string(ImuObs::type()), "",
                           std::string(KvhObs::type()), "", } },
            // clang-format on
            { CONFIG_BOOL, "Use Fixed\nStart Time", "Use the Time configured here as start time", { "0" } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_EMPTY, "", "", {}), ConfigOptionsBase(CONFIG_INT, "Gps Cycle", "GPS Cycle at the beginning of the data recording", { "0", "0", "10" }) } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_EMPTY, "", "", {}), ConfigOptionsBase(CONFIG_INT, "Gps Week", "GPS Week at the beginning of the data recording", { "0", "0", "245760" }) } },
            { CONFIG_VARIANT, "", "", { ConfigOptionsBase(CONFIG_EMPTY, "", "", {}), ConfigOptionsBase(CONFIG_FLOAT, "Gps Time\nof Week", "GPS Time of Week at the beginning of the data recording", { "0", "0", "604800", "3" }) } },
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
    [[nodiscard]] constexpr std::pair<std::string_view, std::string_view> dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return std::make_pair(portDataType, std::string_view(""));
            }
            if (portIndex == 1)
            {
                return std::make_pair(InsObs::type(), std::string_view(""));
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return std::make_pair(portDataType, std::string_view(""));
            }
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
            if (portDataType == VectorNavObs::type())
            {
                auto obs = std::static_pointer_cast<VectorNavObs>(data);
                if (syncVectorNavObs(obs))
                {
                    invokeCallbacks(obs);
                }
            }
            else if (portDataType == ImuObs::type())
            {
                auto obs = std::static_pointer_cast<ImuObs>(data);
                if (syncImuObs(obs))
                {
                    invokeCallbacks(obs);
                }
            }
            else if (portDataType == KvhObs::type())
            {
                auto obs = std::static_pointer_cast<KvhObs>(data);
                if (syncKvhObs(obs))
                {
                    invokeCallbacks(obs);
                }
            }
        }
        else if (portIndex == 1)
        {
            auto obs = std::static_pointer_cast<InsObs>(data);
            syncTime(obs);
        }
    }

    /// @brief Requests the node to send out its data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) final
    {
        if (useFixedStartTime && portIndex == 0)
        {
            // portIndex is the output Port
            // but the data we want to pull come from input port 0
            const auto& sourceNode = incomingLinks[0].first.lock();
            auto& sourcePortIndex = incomingLinks[0].second;

            if (portDataType == VectorNavObs::type())
            {
                auto data = std::static_pointer_cast<VectorNavObs>(sourceNode->requestOutputData(sourcePortIndex));
                if (syncVectorNavObs(data))
                {
                    return data;
                }
            }
            else if (portDataType == ImuObs::type())
            {
                auto data = std::static_pointer_cast<ImuObs>(sourceNode->requestOutputData(sourcePortIndex));
                if (syncImuObs(data))
                {
                    return data;
                }
            }
            else if (portDataType == KvhObs::type())
            {
                auto data = std::static_pointer_cast<KvhObs>(sourceNode->requestOutputData(sourcePortIndex));
                if (syncKvhObs(data))
                {
                    return data;
                }
            }
        }

        return nullptr;
    }

    /// @brief Requests the node to peek its output data
    /// @param[in] portIndex The output port index
    /// @return The requested data or nullptr if no data available
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t portIndex) final
    {
        if (useFixedStartTime && portIndex == 0)
        {
            // portIndex is the output Port
            // but the data we want to pull come from input port 0
            const auto& sourceNode = incomingLinks[0].first.lock();
            auto& sourcePortIndex = incomingLinks[0].second;
            if (portDataType == VectorNavObs::type())
            {
                auto peekData = std::static_pointer_cast<VectorNavObs>(sourceNode->requestOutputDataPeek(sourcePortIndex));
                if (syncVectorNavObs(peekData))
                {
                    return peekData;
                }
            }
            else if (portDataType == ImuObs::type())
            {
                auto peekData = std::static_pointer_cast<ImuObs>(sourceNode->requestOutputDataPeek(sourcePortIndex));
                if (syncImuObs(peekData))
                {
                    return peekData;
                }
            }
            else if (portDataType == KvhObs::type())
            {
                auto peekData = std::static_pointer_cast<KvhObs>(sourceNode->requestOutputDataPeek(sourcePortIndex));
                if (syncKvhObs(peekData))
                {
                    return peekData;
                }
            }
        }

        return nullptr;
    }

  private:
    /// @brief Gets the gps time
    /// @param[in] obs InsObs to process
    void syncTime(std::shared_ptr<InsObs>& obs);

    /// @brief Updates VectorNav Observations with gps time and calls callbacks
    /// @param[in] obs VectorNavObs to process
    /// @return True if the time was updated
    bool syncVectorNavObs(std::shared_ptr<VectorNavObs>& obs);

    /// @brief Updates ImuObs Observations with gps time and calls callbacks
    /// @param[in] obs ImuObs to process
    /// @return True if the time was updated
    bool syncImuObs(std::shared_ptr<ImuObs>& obs);

    /// @brief Updates Kvh Observations with gps time and calls callbacks
    /// @param[in] obs KvhObs to process
    /// @return True if the time was updated
    bool syncKvhObs(std::shared_ptr<KvhObs>& obs);

    /// Input and output Data Types
    std::string_view portDataType;

    /// Flag whether to use the provided start time or wait for a signal on the input port
    bool useFixedStartTime = false;

    /// GPS Time when the sync happened
    std::optional<InsTime> startupGpsTime;

    /// KVH specific variable
    std::optional<uint8_t> prevSequenceNumber;
    /// Time Sync depends on Imu Startup Time
    std::optional<uint64_t> startupImuTime;
};

} // namespace NAV
