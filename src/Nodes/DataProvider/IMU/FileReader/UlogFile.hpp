/// @file UlogFile.hpp
/// @brief File Reader for ULog files
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-12-28

#pragma once

// Already in 'FileReader.hpp'
// #include <string>
// #include <vector>
// #include <fstream>

#include <map>
#include <variant>

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

    /// Combined (sensor-)message name with unique ID
    struct SubscriptionData
    {
        uint8_t multi_id;
        std::string message_name;
    };

    /// The sensor startup UTC time in [µs]
    uint64_t sensorStartupUTCTime_usec{};

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 0;    ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_PosVelAtt = 1; ///< @brief Flow (PosVelAtt)

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

    /// Key-value pair of the message format
    struct DataField
    {
        std::string type; ///< e.g. "uint64_t"
        std::string name; ///< e.g. "timestamp"
    };

    /// Key: message_name, e.g. "sensor_accel"
    std::unordered_map<std::string, std::vector<DataField>> messageFormats;

    /// Px4 acceleration sensor message
    struct SensorAccel
    {
        uint64_t timestamp;        ///< Px4 accelerometer time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t device_id;        ///< unique device identifier
        float x;                   ///< Px4 acceleration along x in p-frame [m/s^2]
        float y;                   ///< Px4 acceleration along y in p-frame [m/s^2]
        float z;                   ///< Px4 acceleration along z in p-frame [m/s^2]
        float temperature;         ///< Px4 temperature of accel sensor in [°C]
        uint32_t error_count;
        std::array<uint8_t, 3> clip_counter;

        static constexpr uint8_t padding = 5;
    };

    /// Px4 gyro sensor message
    struct SensorGyro
    {
        uint64_t timestamp;        ///< Px4 gyroscope time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t device_id;        ///< unique device identifier
        float x;                   ///< Px4 rotation rate about x in p-frame [//TODO]
        float y;                   ///< Px4 rotation rate about y in p-frame [//TODO]
        float z;                   ///< Px4 rotation rate about z in p-frame [//TODO]
        float temperature;         ///< Px4 temperature of gyro sensor in [°C]
        uint32_t error_count;
    };

    /// Px4 magnetometer sensor message
    struct SensorMag
    {
        uint64_t timestamp;        ///< Px4 magnetometer time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t device_id;        ///< unique device identifier
        float x;                   ///< Px4 magnetic flux density about x in p-frame [//TODO]
        float y;                   ///< Px4 magnetic flux density about y in p-frame [//TODO]
        float z;                   ///< Px4 magnetic flux density about z in p-frame [//TODO]
        float temperature;         ///< Px4 temperature of gyro sensor in [°C]
        uint32_t error_count;
        bool is_external;

        static constexpr uint8_t padding = 7;
    };

    /// Px4 GPS sensor message
    struct VehicleGpsPosition
    {
        uint64_t timestamp;     ///< Px4 GPS sensor time since startup in [µs]
        uint64_t time_utc_usec; ///< Px4 GPS UTC time in [µs]
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

        static constexpr uint8_t padding = 5;
    };

    /// Px4 GPS attitude message
    struct VehicleAttitude
    {
        uint64_t timestamp;     ///< Px4 GPS sensor time since startup in [µs]
        std::array<float, 4> q; ///< Px4 GPS attitude quaternion
        std::array<float, 4> delta_q_reset;
        uint8_t quat_reset_counter;

        static constexpr uint8_t padding = 7;
    };

    /// Px4 air data sensor message
    struct VehicleAirData
    {
        uint64_t timestamp;        ///< Px4 air data sensor time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t baro_device_id;   ///< unique device identifier
        float baro_alt_meter;      ///< Px4 barometric altitude in [m]
        float baro_temp_celcius;   ///< Px4 barometric temperature in [°C]
        float baro_pressure_pa;    ///< Px4 barometric pressure in [Pa]
        float rho;
        std::array<uint8_t, 4> _padding0;
    };

    /// Px4 control data message
    struct VehicleControlMode
    {
        uint64_t timestamp; ///< Px4 controller time since startup in [µs]
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

        static constexpr uint8_t padding = 7;
    };

    /// Px4 vehicle status message
    struct VehicleStatus
    {
        uint64_t timestamp;           ///< [µs]
        uint64_t nav_state_timestamp; ///< [µs]
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

    /// Px4 CPU status message
    struct Cpuload
    {
        uint64_t timestamp; ///< Px4 CPU time since startup in [µs]
        float load;         ///< Px4 CPU load
        float ram_usage;    ///< Px4 RAM usage
    };

    /// Key: msg_id
    std::unordered_map<uint16_t, SubscriptionData> subscribedMessages;

    /// Combined  (sensor-)message name with unique ID and data
    struct MeasurementData
    {
        uint8_t multi_id;
        std::string message_name;
        std::variant<SensorAccel, SensorGyro, SensorMag, VehicleGpsPosition, VehicleAttitude> data;
    };

    /// @brief Number of messages read
    uint32_t messageCount = 0;

    /// @brief First absolute time is available (e.g. from GPS)
    bool firstAbsoluteTime = false;

    // Key: [timestamp], Value: [0, "sensor_accel", SensorAccel{}]
    std::multimap<uint64_t, MeasurementData> epochData;

    /// @brief Iterator to reversly loop through epochData
    std::multimap<uint64_t, NAV::UlogFile::MeasurementData>::reverse_iterator it;

    /// @brief Checks 'epochData' whether there is enough data available to output one ImuObs
    /// @param[in] dataMap
    bool enoughImuDataAvailable(std::multimap<uint64_t, MeasurementData> dataMap);

    /// @brief Flag to check whether 'epochData' contains accelerometer reading
    bool holdsAccel = false;
    /// @brief Flag to check whether 'epochData' contains gyro reading
    bool holdsGyro = false;
    /// @brief Flag to check whether 'epochData' contains magnetometer reading
    bool holdsMag = false;

    /// @brief Timestamp of the latest accelerometer reading
    uint64_t accelKey{};
    /// @brief Timestamp of the latest gyro reading
    uint64_t gyroKey{};
    /// @brief Timestamp of the latest magnetometer reading
    uint64_t magKey{};
};
} // namespace NAV