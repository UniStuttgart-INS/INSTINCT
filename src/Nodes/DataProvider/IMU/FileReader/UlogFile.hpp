/// @file UlogFile.hpp
/// @brief File Reader for ULog files
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-12-28

#pragma once

// Already in 'FileReader.hpp'
// #include <string>
// #include <vector>
// #include <fstream>

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

namespace NAV
{
class UlogFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    UlogFile();
    /// @brief Destructor
    ~UlogFile() override;
    /// @brief Copy constructor
    UlogFile(const UlogFile&) = delete;
    /// @brief Move constructor
    UlogFile(UlogFile&&) = delete;
    /// @brief Copy assignment operator
    UlogFile& operator=(const UlogFile&) = delete;
    /// @brief Move assignment operator
    UlogFile& operator=(UlogFile&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

    /// Key-value pair of the message format
    struct DataField
    {
        std::string type; ///< e.g. "uint64_t"
        std::string name; ///< e.g. "timestamp"
    };

    /// Key: message_name, e.g. "sensor_accel"
    std::map<std::string, std::vector<DataField>> messageFormats;

    struct SensorAccel
    {
        uint64_t timestamp;
        uint64_t timestamp_sample;
        uint32_t device_id;
        float x;
        float y;
        float z;
        float temperature;
        uint32_t error_count;
        std::array<uint8_t, 3> clip_counter;
        std::array<uint8_t, 5> _padding0;
    };

    struct SensorGyro
    {
        uint64_t timestamp;
        uint64_t timestamp_sample;
        uint32_t device_id;
        float x;
        float y;
        float z;
        float temperature;
        uint32_t error_count;
    };

    struct SensorMag
    {
        uint64_t timestamp;
        uint64_t timestamp_sample;
        uint32_t device_id;
        float x;
        float y;
        float z;
        float temperature;
        uint32_t error_count;
        bool is_external;
        std::array<uint8_t, 7> _padding0;
    };

    //TODO: Declare structs for other sensors, e.g. GPS, etc. (in header)

    /// Combined (sensor-)message name with unique ID
    struct SubscriptionData
    {
        uint8_t multi_id;
        std::string message_name;
    };

    /// Key: msg_id
    std::map<uint16_t, SubscriptionData> subscribedMessages;

    uint64_t currentTimestamp{};

  private:
    constexpr static size_t OutputPortIndex_UlogOutput = 0; ///< @brief Flow (UlogOutput)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData([[maybe_unused]] bool peek = false); //TODO: remove [[maybe_unused]] when enabling the callbacks

    /// @brief Determines the type of the file
    /// @return The file type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Read the ULog Definitions, i.e. a variable length section containing version information, format definitions and (initial) parameter values (see https://docs.px4.io/master/en/dev_log/ulog_file_format.html#definitions-section)
    void readDefinitions();

    //TODO: Replace with 'pollData', once callbacks are enabled
    void readData();

    /// @brief Read msg type 'I'
    /// @param[in] msgSize size of ulogMsgHeader
    /// @param[in] msgType type of ulogMsgHeader
    void readInformationMessage(uint16_t msgSize, char msgType);

    /// @brief Read msg type 'M'
    /// @param[in] msgSize size of ulogMsgHeader
    /// @param[in] msgType type of ulogMsgHeader
    void readInformationMessageMulti(uint16_t msgSize, char msgType);

    /// @brief Read msg type 'P'
    /// @param[in] msgSize size of ulogMsgHeader
    /// @param[in] msgType type of ulogMsgHeader
    void readParameterMessage(uint16_t msgSize, char msgType);

    /// @brief Read msg type 'Q'
    /// @param[in] msgSize size of ulogMsgHeader
    /// @param[in] msgType type of ulogMsgHeader
    void readParameterMessageDefault(uint16_t msgSize, char msgType);

    /// @brief Number of messages read
    uint32_t messageCount = 0;
};
} // namespace NAV