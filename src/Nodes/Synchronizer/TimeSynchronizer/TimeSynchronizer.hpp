/**
 * @file TimeSynchronizer.hpp
 * @brief Class to Synchronize Different Data Providers to the same Time base
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-21
 */

#pragma once

#include "Nodes/Node.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Class to Synchronize Different Data Providers to the same Time base
class TimeSynchronizer final : public Node
{
  public:
    /**
     * @brief Construct a new Time Synchronizer object
     * 
     * @param[in] name Name of the Object
     * @param[in] options Program options string map
     */
    TimeSynchronizer(const std::string& name, const std::map<std::string, std::string>& options);

    TimeSynchronizer() = default;                                  ///< Default Constructor
    ~TimeSynchronizer() final = default;                           ///< Destructor
    TimeSynchronizer(const TimeSynchronizer&) = delete;            ///< Copy constructor
    TimeSynchronizer(TimeSynchronizer&&) = delete;                 ///< Move constructor
    TimeSynchronizer& operator=(const TimeSynchronizer&) = delete; ///< Copy assignment operator
    TimeSynchronizer& operator=(TimeSynchronizer&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("TimeSynchronizer");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("TimeSync");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<ConfigOptions> The gui configuration
     */
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        return { { CONFIG_BOOL, "Use Fixed\nStart Time", "Use the Time configured here as start time", { "0" } },
                 { CONFIG_INT, "Gps Cycle", "GPS Cycle at the beginning of the data recording", { "0", "0", "10" } },
                 { CONFIG_INT, "Gps Week", "GPS Week at the beginning of the data recording", { "0", "0", "245760" } },
                 { CONFIG_FLOAT, "Gps Time\nof Week", "GPS Time of Week at the beginning of the data recording", { "0", "0", "604800" } } };
    }

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::ALL;
    }

    /**
     * @brief Returns the number of Ports
     * 
     * @param[in] portType Specifies the port type
     * @retval constexpr uint8_t The number of ports
     */
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

    /**
     * @brief Returns the data types provided by this class
     * 
     * @param[in] portType Specifies the port type
     * @param[in] portIndex Port index on which the data is sent
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return InsObs().type();
            }
            if (portIndex == 1)
            {
                return VectorNavObs().type();
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return VectorNavObs().type();
            }
        }

        return std::string_view("");
    }

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<InsObs>(data);
            syncTime(obs);
        }
        else if (portIndex == 1)
        {
            auto obs = std::static_pointer_cast<VectorNavObs>(data);
            syncVectorNavSensor(obs);
        }
    }
    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) final
    {
        if (useFixedStartTime && portIndex == 0)
        {
            // portIndex is the output Port
            // but the data we want to pull come from input port 1
            const auto& sourceNode = incomingLinks[1].first.lock();
            auto& sourcePortIndex = incomingLinks[1].second;

            auto data = std::static_pointer_cast<VectorNavObs>(sourceNode->requestOutputData(sourcePortIndex));
            if (updateInsTime(data))
            {
                return data;
            }
        }

        return nullptr;
    }

    /**
     * @brief Requests the node to peek its output data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t portIndex) final
    {
        if (useFixedStartTime && portIndex == 0)
        {
            // portIndex is the output Port
            // but the data we want to pull come from input port 1
            const auto& sourceNode = incomingLinks[1].first.lock();
            auto& sourcePortIndex = incomingLinks[1].second;

            auto peekData = std::static_pointer_cast<VectorNavObs>(sourceNode->requestOutputDataPeek(sourcePortIndex));
            if (updateInsTime(peekData))
            {
                return peekData;
            }
        }

        return nullptr;
    }

  private:
    /**
     * @brief Updates VectorNav Observations with gps time
     * 
     * @param[in, out] obs VectorNavObs to process
     * @retval bool Returns true if time was updated
     */
    bool updateInsTime(std::shared_ptr<VectorNavObs>& obs);

    /**
     * @brief Gets the gps time from an UbloxSensor
     * 
     * @param[in] obs InsObs to process
     */
    void syncTime(std::shared_ptr<InsObs>& obs);

    /**
     * @brief Updates VectorNav Observations with gps time and calls callbacks
     * 
     * @param[in] obs VectorNavObs to process
     */
    void syncVectorNavSensor(std::shared_ptr<VectorNavObs>& obs);

    bool useFixedStartTime = false;

    std::optional<InsTime> startupGpsTime;
    std::optional<uint64_t> startupImuTime;
};

} // namespace NAV
