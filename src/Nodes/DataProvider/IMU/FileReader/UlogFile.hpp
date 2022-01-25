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

    struct VehicleGpsPosition
    {
        uint64_t timestamp;
        uint64_t time_utc_usec;
        int32_t lat;
        int32_t lon;
        int32_t alt;
        int32_t alt_ellipsoid;
        float s_variance_m_s;
        float c_variance_rad;
        float eph;
        float epv;
        float hdop;
        float vdop;
        int32_t noise_per_ms;
        int32_t jamming_indicator;
        float vel_m_s;
        float vel_n_m_s;
        float vel_e_m_s;
        float vel_d_m_s;
        float cog_rad;
        int32_t timestamp_time_relative;
        float heading;
        float heading_offset;
        uint8_t fix_type;
        bool vel_ned_valid;
        uint8_t satellites_used;
        std::array<uint8_t, 5> _padding0;
    };

    struct VehicleAttitude
    {
        uint64_t timestamp;
        std::array<float, 4> q;
        std::array<float, 4> delta_q_reset;
        uint8_t quat_reset_counter;
        std::array<uint8_t, 7> _padding0;
    };

    struct VehicleAirData
    {
        uint64_t timestamp;
        uint64_t timestamp_sample;
        uint32_t baro_device_id;
        float baro_alt_meter;
        float baro_temp_celcius;
        float baro_pressure_pa;
        float rho;
        std::array<uint8_t, 4> _padding0;
    };

    struct VehicleControlMode
    {
        uint64_t timestamp;
        bool flag_armed;
        bool flag_external_manual_override_ok;
        bool flag_control_manual_enabled;
        bool flag_control_auto_enabled;
        bool flag_control_offboard_enabled;
        bool flag_control_rates_enabled;
        bool flag_control_attitude_enabled;
        bool flag_control_yawrate_override_enabled;
        bool flag_control_rattitude_enabled;
        bool flag_control_force_enabled;
        bool flag_control_acceleration_enabled;
        bool flag_control_velocity_enabled;
        bool flag_control_position_enabled;
        bool flag_control_altitude_enabled;
        bool flag_control_climb_rate_enabled;
        bool flag_control_termination_enabled;
        bool flag_control_fixed_hdg_enabled;
        std::array<uint8_t, 7> _padding0;
    };

    struct VehicleStatus
    {
        uint64_t timestamp;
        uint64_t nav_state_timestamp;
        uint32_t onboard_control_sensors_present;
        uint32_t onboard_control_sensors_enabled;
        uint32_t onboard_control_sensors_health;
        uint8_t nav_state;
        uint8_t arming_state;
        uint8_t hil_state;
        bool failsafe;
        uint8_t system_type;
        uint8_t system_id;
        uint8_t component_id;
        uint8_t vehicle_type;
        bool is_vtol;
        bool is_vtol_tailsitter;
        bool vtol_fw_permanent_stab;
        bool in_transition_mode;
        bool in_transition_to_fw;
        bool rc_signal_lost;
        uint8_t rc_input_mode;
        bool data_link_lost;
        uint8_t data_link_lost_counter;
        bool high_latency_data_link_lost;
        bool engine_failure;
        bool mission_failure;
        uint8_t failure_detector_status;
        uint8_t latest_arming_reason;
        uint8_t latest_disarming_reason;
        std::array<uint8_t, 5> _padding0;
    };

    struct Cpuload
    {
        uint64_t timestamp;
        float load;
        float ram_usage;
    };

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