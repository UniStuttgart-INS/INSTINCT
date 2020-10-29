/// @file KvhFile.hpp
/// @brief File Reader for Kvh log files
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "../ImuFileReader.hpp"
#include "NodeData/IMU/KvhObs.hpp"

#include "util/UartSensors/KVH/KvhUartSensor.hpp"

namespace NAV
{
/// File Reader for Kvh log files
class KvhFile final : public ImuFileReader
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Sensor which wrote the file
    /// @param[in] options Program options string map
    KvhFile(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    KvhFile() = default;
    /// @brief Destructor
    ~KvhFile() final = default;
    /// @brief Copy constructor
    KvhFile(const KvhFile&) = delete;
    /// @brief Move constructor
    KvhFile(KvhFile&&) = delete;
    /// @brief Copy assignment operator
    KvhFile& operator=(const KvhFile&) = delete;
    /// @brief Move assignment operator
    KvhFile& operator=(KvhFile&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("KvhFile");
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
            return 2U;
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
                return std::make_pair(KvhObs().type(), std::string_view(""));
            }
            if (portIndex == 1)
            {
                return std::make_pair(ImuPos().type(), std::string_view(""));
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
    /// @brief Polls the data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<KvhObs> pollData(bool peek = false);

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() final;

    /// Sensor Object
    sensors::kvh::KvhUartSensor sensor;

    /// Previous Sequence number to check for order errors
    uint8_t prevSequenceNumber = UINT8_MAX;
};

} // namespace NAV