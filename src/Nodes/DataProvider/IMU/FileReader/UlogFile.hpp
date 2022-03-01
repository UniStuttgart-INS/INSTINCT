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
/// @brief File Reader for ULog files ('.ulg')
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

    /// @brief Combined (sensor-)message name with unique ID
    struct SubscriptionData
    {
        uint8_t multi_id;         ///< the same message format can have multiple instances, for example if the system has two sensors of the same type. The default and first instance must be 0
        std::string message_name; ///< message name to subscribe to
    };

    /// @brief The sensor startup UTC time in [µs]
    uint64_t sensorStartupUTCTime_usec{};

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 0; ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_PosVel = 1; ///< @brief Flow (PosVelAtt)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData(bool peek = false);

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

    /// @brief Key-value pair of the message format
    struct DataField
    {
        std::string type; ///< e.g. "uint64_t"
        std::string name; ///< e.g. "timestamp"
    };

    /// @brief Key: message_name, e.g. "sensor_accel"
    std::unordered_map<std::string, std::vector<DataField>> messageFormats;

    /// @brief Px4 acceleration sensor message
    struct SensorAccel
    {
        uint64_t timestamp;                  ///< Px4 accelerometer time since startup in [µs]
        uint64_t timestamp_sample;           ///< [µs]
        uint32_t device_id;                  ///< unique device identifier
        float x;                             ///< Px4 acceleration along x in p-frame [m/s^2]
        float y;                             ///< Px4 acceleration along y in p-frame [m/s^2]
        float z;                             ///< Px4 acceleration along z in p-frame [m/s^2]
        float temperature;                   ///< Px4 temperature of accel sensor in [°C]
        uint32_t error_count;                ///< error count
        std::array<uint8_t, 3> clip_counter; ///< clip counter

        static constexpr uint8_t padding = 5; ///< padding
    };

    /// @brief Px4 gyro sensor message
    struct SensorGyro
    {
        uint64_t timestamp;        ///< Px4 gyroscope time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t device_id;        ///< unique device identifier
        float x;                   ///< Px4 rotation rate about x in p-frame [//TODO]
        float y;                   ///< Px4 rotation rate about y in p-frame [//TODO]
        float z;                   ///< Px4 rotation rate about z in p-frame [//TODO]
        float temperature;         ///< Px4 temperature of gyro sensor in [°C]
        uint32_t error_count;      ///< error count
    };

    /// @brief Px4 magnetometer sensor message
    struct SensorMag
    {
        uint64_t timestamp;        ///< Px4 magnetometer time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t device_id;        ///< unique device identifier
        float x;                   ///< Px4 magnetic flux density about x in p-frame [//TODO]
        float y;                   ///< Px4 magnetic flux density about y in p-frame [//TODO]
        float z;                   ///< Px4 magnetic flux density about z in p-frame [//TODO]
        float temperature;         ///< Px4 temperature of gyro sensor in [°C]
        uint32_t error_count;      ///< error count
        bool is_external;          ///< Flag

        static constexpr uint8_t padding = 7; ///< padding
    };

    /// @brief Px4 GPS sensor message
    struct VehicleGpsPosition
    {
        uint64_t timestamp;              ///< Px4 GPS sensor time since startup in [µs]
        uint64_t time_utc_usec;          ///< Px4 GPS UTC time in [µs]
        int32_t lat;                     ///< Latitude in [//TODO]
        int32_t lon;                     ///< Longitude in [//TODO]
        int32_t alt;                     ///< Altitude above ground in [m]
        int32_t alt_ellipsoid;           ///< Altitude above ellipsoid in [m]
        float s_variance_m_s;            ///< Variance of speed
        float c_variance_rad;            ///< Variance of angle
        float eph;                       ///< Horizontal position error in [m]
        float epv;                       ///< Vertical position error in [m]
        float hdop;                      ///< Horizontal dilusion of precision
        float vdop;                      ///< Vertical dilusion of precision
        int32_t noise_per_ms;            ///< Noise per millisecond
        int32_t jamming_indicator;       ///< Jamming indicator
        float vel_m_s;                   ///< Velocity in [m/s]
        float vel_n_m_s;                 ///< Velocity north component in [m/s]
        float vel_e_m_s;                 ///< Velocity east component in [m/s]
        float vel_d_m_s;                 ///< Velocity down component in [m/s]
        float cog_rad;                   ///< Center of gravity
        int32_t timestamp_time_relative; ///< Relative time stamp
        float heading;                   ///< heading
        float heading_offset;            ///< heading offset
        uint8_t fix_type;                ///< fix type
        bool vel_ned_valid;              ///< Flag for validation of velocity in NED
        uint8_t satellites_used;         ///< # of satellites used

        static constexpr uint8_t padding = 5; ///< padding
    };

    /// @brief Px4 GPS attitude message
    struct VehicleAttitude
    {
        uint64_t timestamp;                 ///< Px4 GPS sensor time since startup in [µs]
        std::array<float, 4> q;             ///< Px4 GPS attitude quaternion
        std::array<float, 4> delta_q_reset; ///< delta q reset
        uint8_t quat_reset_counter;         ///< Quaternion reset counter

        static constexpr uint8_t padding = 7; ///< padding
    };

    /// @brief Px4 air data sensor message
    struct VehicleAirData
    {
        uint64_t timestamp;        ///< Px4 air data sensor time since startup in [µs]
        uint64_t timestamp_sample; ///< [µs]
        uint32_t baro_device_id;   ///< unique device identifier
        float baro_alt_meter;      ///< Px4 barometric altitude in [m]
        float baro_temp_celcius;   ///< Px4 barometric temperature in [°C]
        float baro_pressure_pa;    ///< Px4 barometric pressure in [Pa]
        float rho;                 ///< density?

        static constexpr uint8_t padding = 4; ///< padding
    };

    /// @brief Px4 control data message
    struct VehicleControlMode
    {
        uint64_t timestamp;                         ///< Px4 controller time since startup in [µs]
        bool flag_armed;                            ///< Flag: Arm switch
        bool flag_external_manual_override_ok;      ///< Flag: external manual override ok
        bool flag_control_manual_enabled;           ///< Flag: manual mode enabled
        bool flag_control_auto_enabled;             ///< Flag: auto mode enabled
        bool flag_control_offboard_enabled;         ///< Flag: offboard enabled
        bool flag_control_rates_enabled;            ///< Flag: rates enabled
        bool flag_control_attitude_enabled;         ///< Flag: attitude mode enabled
        bool flag_control_yawrate_override_enabled; ///< Flag: yawrate override enabled
        bool flag_control_rattitude_enabled;        ///< Flag: rattitude enabled
        bool flag_control_force_enabled;            ///< Flag: force enabled
        bool flag_control_acceleration_enabled;     ///< Flag: acceleration enabled
        bool flag_control_velocity_enabled;         ///< Flag: velocity enabled
        bool flag_control_position_enabled;         ///< Flag: position enabled
        bool flag_control_altitude_enabled;         ///< Flag: altitude enabled
        bool flag_control_climb_rate_enabled;       ///< Flag: climb rate enabled
        bool flag_control_termination_enabled;      ///< Flag: termination enabled
        bool flag_control_fixed_hdg_enabled;        ///< Flag: fixed heading enabled

        static constexpr uint8_t padding = 7; ///< padding
    };

    /// @brief Px4 vehicle status message
    struct VehicleStatus
    {
        uint64_t timestamp;                       ///< [µs]
        uint64_t nav_state_timestamp;             ///< [µs]
        uint32_t onboard_control_sensors_present; ///< onboard control sensors present
        uint32_t onboard_control_sensors_enabled; ///< onboard control sensors enabled
        uint32_t onboard_control_sensors_health;  ///< onboard control sensors health
        uint8_t nav_state;                        ///< nav state
        uint8_t arming_state;                     ///< arming state
        uint8_t hil_state;                        ///< hil state
        bool failsafe;                            ///< Flag: failsafe
        uint8_t system_type;                      ///< system type
        uint8_t system_id;                        ///< system id
        uint8_t component_id;                     ///< component id
        uint8_t vehicle_type;                     ///< vehicle type
        bool is_vtol;                             ///< Flag: is vertical take-off and landing
        bool is_vtol_tailsitter;                  ///< Flag: is vertical take-off and landing tailsitter
        bool vtol_fw_permanent_stab;              ///< Flag: vertical take-off and landing fw permanent stability
        bool in_transition_mode;                  ///< Flag: transition mode
        bool in_transition_to_fw;                 ///< Flag: transition to fw
        bool rc_signal_lost;                      ///< Flag: RC signal lost
        uint8_t rc_input_mode;                    ///< RC input mode
        bool data_link_lost;                      ///< Flag: Data link lost
        uint8_t data_link_lost_counter;           ///< Counter how often data link was lost
        bool high_latency_data_link_lost;         ///< Flag: high latency data link lost
        bool engine_failure;                      ///< Flag: engine failure
        bool mission_failure;                     ///< Flag: mission failure
        uint8_t failure_detector_status;          ///< failure detector status
        uint8_t latest_arming_reason;             ///< latest arming reason
        uint8_t latest_disarming_reason;          ///< latest disarming reason
        std::array<uint8_t, 5> _padding0;         ///< padding
    };

    /// @brief Px4 CPU status message
    struct Cpuload
    {
        uint64_t timestamp; ///< Px4 CPU time since startup in [µs]
        float load;         ///< Px4 CPU load
        float ram_usage;    ///< Px4 RAM usage
    };

    /// @brief Key: msg_id
    std::unordered_map<uint16_t, SubscriptionData> subscribedMessages;

    /// @brief Combined  (sensor-)message name with unique ID and data
    struct MeasurementData
    {
        uint8_t multi_id;                                                                           ///< multiple instances of the same message format, for example if the system has two sensors of the same type. The default and first instance must be 0
        std::string message_name;                                                                   ///< message name to subscribe to
        std::variant<SensorAccel, SensorGyro, SensorMag, VehicleGpsPosition, VehicleAttitude> data; ///< measurement data
    };

    /// @brief Number of messages read
    uint32_t messageCount = 0;

    /// @brief First absolute time is available (e.g. from GPS)
    bool firstAbsoluteTime = false;

    /// @brief Data message container. Key: [timestamp], Value: [0, "sensor_accel", SensorAccel{}]
    std::multimap<uint64_t, MeasurementData> epochData;

    /// @brief Iterator to reversly loop through epochData
    std::multimap<uint64_t, NAV::UlogFile::MeasurementData>::reverse_iterator it;

    /// @brief Checks 'epochData' whether there is enough data available to output one ImuObs
    /// @param[in] dataMap Multimap that contains measurement data
    bool enoughImuDataAvailable(std::multimap<uint64_t, MeasurementData> dataMap);

    /// @brief Flag to check whether 'epochData' contains accelerometer reading
    bool holdsAccel = false;
    /// @brief Flag to check whether 'epochData' contains gyro reading
    bool holdsGyro = false;
    /// @brief Flag to check whether 'epochData' contains magnetometer reading
    bool holdsMag = false;
    /// @brief Flag to check whether 'epochData' contains GPS reading
    bool holdsGps = false;

    /// @brief Timestamp of the latest accelerometer reading
    uint64_t accelKey{};
    /// @brief Timestamp of the latest gyro reading
    uint64_t gyroKey{};
    /// @brief Timestamp of the latest magnetometer reading
    uint64_t magKey{};

    /// @brief Flag to check whether loop is run again without re-initialization
    bool isReRun = false;

    /// Stores GNSS timestamp of one epoch before the current one (relative or absolute)
    struct
    {
        uint64_t timeSinceStartup{}; ///< Relative timestamp
        InsTime gnssTime;            ///< Absolute timestamp
    } lastGnssTime;
};
} // namespace NAV