/// @file EmlidFile.hpp
/// @brief File Reader for Emlid log files
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "../GnssFileReader.hpp"
#include "NodeData/GNSS/EmlidObs.hpp"

#include "util/UartSensors/Emlid/EmlidUartSensor.hpp"

namespace NAV
{
/// File Reader for Emlid log files
class EmlidFile final : public GnssFileReader
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Node
    /// @param[in] options Program options string map
    EmlidFile(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    EmlidFile() = default;
    /// @brief Destructor
    ~EmlidFile() final = default;
    /// @brief Copy constructor
    EmlidFile(const EmlidFile&) = delete;
    /// @brief Move constructor
    EmlidFile(EmlidFile&&) = delete;
    /// @brief Copy assignment operator
    EmlidFile& operator=(const EmlidFile&) = delete;
    /// @brief Move assignment operator
    EmlidFile& operator=(EmlidFile&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("EmlidFile");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataProvider");
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
            break;
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
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                return std::make_pair(EmlidObs::type(), std::string_view(""));
            }
        }

        return std::make_pair(std::string_view(""), std::string_view(""));
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
    /// @brief Polls the data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<EmlidObs> pollData(bool peek = false);

    /// @brief Determines the type of the file (ASCII or binary)
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() final;

    /// Sensor Object
    sensors::emlid::EmlidUartSensor sensor;
};

} // namespace NAV
