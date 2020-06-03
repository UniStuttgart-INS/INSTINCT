/**
 * @file VectorNavFile.hpp
 * @brief File Reader for Vector Nav log files
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-16
 */

#pragma once

#include "../Imu.hpp"
#include "../../Protocol/FileReader.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

namespace NAV
{
/// File Reader for Vector Nav log files
class VectorNavFile final : public FileReader, public Imu
{
  public:
    /**
     * @brief Construct a new Vector Nav File object
     * 
     * @param[in] name Name of the Sensor which wrote the file
     * @param[in, out] options Program options string list
     */
    VectorNavFile(const std::string& name, std::deque<std::string>& options);

    VectorNavFile() = default;                               ///< Default Constructor
    ~VectorNavFile() final;                                  ///< Destructor
    VectorNavFile(const VectorNavFile&) = delete;            ///< Copy constructor
    VectorNavFile(VectorNavFile&&) = delete;                 ///< Move constructor
    VectorNavFile& operator=(const VectorNavFile&) = delete; ///< Copy assignment operator
    VectorNavFile& operator=(VectorNavFile&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("VectorNavFile");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("DataProvider");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> The gui configuration
     */
    [[nodiscard]] std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> guiConfig() const final
    {
        return { { ConfigOptions::CONFIG_STRING, "Path", "Path to the File to read", { "" } } };
    }

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::POST_PROCESSING;
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
            break;
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
    void handleInputData(uint8_t /* portIndex */, std::shared_ptr<NodeData> /* data */) final {}

    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t portIndex) final
    {
        if (portIndex == 0)
        {
            return pollData();
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
        if (portIndex == 0)
        {
            return pollData(true);
        }

        return nullptr;
    }

    /**
     * @brief Resets the node. In case of file readers, that moves the read cursor to the start
     */
    void resetNode() final;

  private:
    /**
     * @brief Polls the data from the file
     * 
     * @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
     * @retval std::shared_ptr<VectorNavObs> The read observation
     */
    [[nodiscard]] std::shared_ptr<VectorNavObs> pollData(bool peek = false);

    /**
     * @brief Determines the type of the file (ASCII or binary)
     * 
     * @retval FileType The File Type
     */
    [[nodiscard]] FileType determineFileType() final;

    /// Header Columns of an ASCII file
    std::vector<std::string> columns;
};

} // namespace NAV
