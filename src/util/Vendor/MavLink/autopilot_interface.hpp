/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.hpp
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

#pragma once

#if __linux__ || __APPLE__
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

    #include "generic_port.hpp"

    #include <signal.h> // NOLINT(hicpp-deprecated-headers,modernize-deprecated-headers) // FIXME: inclusion of deprecated C++ header 'signal.h'; consider using 'csignal' instead
    #include <time.h>   // NOLINT(hicpp-deprecated-headers,modernize-deprecated-headers) // FIXME: inclusion of deprecated C++ header 'time.h'; consider using 'ctime' instead
    #include <sys/time.h>
    #include <pthread.h> // This uses POSIX Threads
    #include <unistd.h>  // UNIX standard function definitions
    #include <mutex>

    #include <mavlink/common/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

// bit number  876543210987654321
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION = 0b0000110111111000;     ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY = 0b0000110111000111;     ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION = 0b0000110000111111; ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE = 0b0000111000111111;        ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE = 0b0000100111111111;    ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE = 0b0000010111111111;     ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE

constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF = 0x1000; ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND = 0x2000;    ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER = 0x3000;  ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE = 0x4000;    ///< MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE

    /// Available Baudrates
    #define AUTOPILOT_MODE_STABILIZED 0   ///< Self-levels the roll and pitch axis
    #define AUTOPILOT_MODE_ACRO 1         ///< Holds attitude, no self-level
    #define AUTOPILOT_MODE_ALTHOLD 2      ///< Holds altitude and self-levels the roll & pitch
    #define AUTOPILOT_MODE_AUTO 3         ///< Executes pre-defined mission
    #define AUTOPILOT_MODE_GUIDED 4       ///< Navigates to single points commanded by GCS
    #define AUTOPILOT_MODE_LOITER 5       ///< Holds altitude and position, uses GPS for movements
    #define AUTOPILOT_MODE_RTL 6          ///< Returns above takeoff location, may also include landing
    #define AUTOPILOT_MODE_CIRCLE 7       ///< Automatically circles a point in front of the vehicle
    #define AUTOPILOT_MODE_LAND 9         ///< Reduces altitude to ground level, attempts to go straight down
    #define AUTOPILOT_MODE_DRIFT 11       ///< Like stabilize, but coordinates yaw with roll like a plane
    #define AUTOPILOT_MODE_SPORT 13       ///< Alt-hold, but holds pitch & roll when sticks centered
    #define AUTOPILOT_MODE_FLIP 14        ///< Rises and completes an automated flip
    #define AUTOPILOT_MODE_AUTOTUNE 15    ///< Automated pitch and bank procedure to improve control loops
    #define AUTOPILOT_MODE_POSHOLD 16     ///< Like loiter, but manual roll and pitch when sticks not centered
    #define AUTOPILOT_MODE_BRAKE 17       ///< Brings copter to an immediate stop
    #define AUTOPILOT_MODE_THROW 18       ///< Holds position after a throwing takeoff
    #define AUTOPILOT_MODE_AVOIDADSB 19   ///< ADS-B based avoidance of manned aircraft
    #define AUTOPILOT_MODE_GUIDEDNOGPS 20 ///< This variation of Guided mode does not require a GPS but it only accepts attitude targets
    #define AUTOPILOT_MODE_SMARTRTL 21    ///< RTL, but traces path to get home
    #define AUTOPILOT_MODE_FOLOWHOLD 22   ///< Position control using Optical Flow
    #define AUTOPILOT_MODE_FOLLOW 23      ///< Follows another vehicle
    #define AUTOPILOT_MODE_ZIGZAG 24      ///< Useful for crop spraying
    #define AUTOPILOT_MODE_SYSTEMID 25    ///< Special diagnostic/modeling mode

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// helper functions

/// @brief Get time in [µs]
uint64_t get_time_usec();

/// @brief Set position in NED coordinates
/// @param[in] x desired position value, north component
/// @param[in] y desired position value, east component
/// @param[in] z desired position value, down component
/// @param[in] sp Mavlink set position target local NED
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t& sp);

/// @brief Set velocity in NED coordinates
/// @param[in] vx desired velocity value, north component
/// @param[in] vy desired velocity value, east component
/// @param[in] vz desired velocity value, down component
/// @param[in] sp Mavlink set position target local NED
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t& sp);

/// @brief Set acceleration in NED coordinates
/// @param[in] ax desired acceleration value, north component
/// @param[in] ay desired acceleration value, east component
/// @param[in] az desired acceleration value, down component
/// @param[in] sp Mavlink set position target local NED
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t& sp);

/// @brief Set yaw angle
/// @param[in] yaw desired yaw angle
/// @param[in] sp Mavlink set position target local NED
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t& sp);

/// @brief Set rate of yaw angle
/// @param[in] yaw_rate desired rate of yaw angle
/// @param[in] sp Mavlink set position target local NED
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t& sp);

/// @brief Start autopilot interface read thread
/// @param[in] args Arguments
void* start_autopilot_interface_read_thread(void* args);

/// @brief Start autopilot interface write thread
/// @param[in] args Arguments
void* start_autopilot_interface_write_thread(void* args);

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

/// @brief Time Stamps
struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat{};                  ///< heartbeat
    uint64_t sys_status{};                 ///< sys_status
    uint64_t battery_status{};             ///< battery_status
    uint64_t radio_status{};               ///< radio_status
    uint64_t local_position_ned{};         ///< local_position_ned
    uint64_t global_position_int{};        ///< global_position_int
    uint64_t position_target_local_ned{};  ///< position_target_local_ned
    uint64_t position_target_global_int{}; ///< position_target_global_int
    uint64_t highres_imu{};                ///< highres_imu
    uint64_t attitude{};                   ///< attitude

    /// @brief Reset the timestamps to zero
    void reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
    }
};

/// @brief Struct containing information on the MAV we are currently connected to
struct Mavlink_Messages
{
    int sysid{};  ///< System ID
    int compid{}; ///< Component ID

    mavlink_heartbeat_t heartbeat{};                                   ///< Heartbeat
    mavlink_sys_status_t sys_status{};                                 ///< System Status
    mavlink_battery_status_t battery_status{};                         ///< Battery Status
    mavlink_radio_status_t radio_status{};                             ///< Radio Status
    mavlink_local_position_ned_t local_position_ned{};                 ///< Local Position
    mavlink_global_position_int_t global_position_int{};               ///< Global Position
    mavlink_position_target_local_ned_t position_target_local_ned{};   ///< Local Position Target
    mavlink_position_target_global_int_t position_target_global_int{}; ///< Global Position Target
    mavlink_highres_imu_t highres_imu{};                               ///< HiRes IMU
    mavlink_attitude_t attitude{};                                     ///< Attitude

    // System Parameters?

    Time_Stamps time_stamps{}; ///< Time Stamps

    /// @brief Reset timestamps
    void reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }
};

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */

/// @brief Autopilot Interface Class
class Autopilot_Interface
{
  public:
    /// @brief Default constructor
    Autopilot_Interface() = default;
    /// @brief Constructor
    explicit Autopilot_Interface(Generic_Port* port_);
    /// @brief Destructor
    ~Autopilot_Interface() = default;
    /// @brief Copy constructor
    Autopilot_Interface(const Autopilot_Interface&) = delete;
    /// @brief Move constructor
    Autopilot_Interface(Autopilot_Interface&&) = delete;
    /// @brief Copy assignment operator
    Autopilot_Interface& operator=(const Autopilot_Interface&) = delete;
    /// @brief Move assignment operator
    Autopilot_Interface& operator=(Autopilot_Interface&&) = delete;

    uint64_t write_count{}; ///< Write count
    char reading_status{};  ///< Reading status
    char writing_status{};  ///< Writing status
    char control_status{};  ///< Control status

    int system_id{};    ///< System ID
    int autopilot_id{}; ///< Autopilot ID
    int companion_id{}; ///< Companion ID

    // MOCAP Variables
    float mocap_w{};            ///< MOCAP quaternion, scalar part
    float mocap_x{};            ///< MOCAP quaternion, vector component x
    float mocap_y{};            ///< MOCAP quaternion, vector component y
    float mocap_z{};            ///< MOCAP quaternion, vector component z
    bool MOCAP_Active = false;  ///< Flag to indicate whether MOCAP is active
    double MOCAP_Frequency = 0; ///< MOCAP frequency

    // GPS Variables
    int32_t gps_lat_d{};         ///< GPS latitude
    int32_t gps_lon_d{};         ///< GPS longitude
    float gps_alt{};             ///< GPS altitude
    float gps_vn{};              ///< GPS velocity north component
    float gps_ve{};              ///< GPS velocity east component
    float gps_vd{};              ///< GPS velocity down component
    uint32_t gps_time_week_ms{}; ///< GPS time of week in [ms]
    uint16_t gps_time_week{};    ///< GPS time of week
    bool GPS_Active = false;     ///< GPS active
    double GPS_Frequency = 0;    ///< GPS frequency

    Mavlink_Messages current_messages{};                        ///< Current Mavlink message
    mavlink_set_position_target_local_ned_t initial_position{}; ///< Initial position for Mavlink set position target local NED

    /// @brief Update setpoint of Mavlink position target in NED
    /// @param[in] setpoint Setpoint of Mavlink position target in NED
    void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);

    /// @brief Read messages
    void read_messages();

    /// @brief Write messages
    /// @param[in] message Mavlink message to write
    int write_message(mavlink_message_t message);

    /// @brief Arm / disarm
    /// @param[in] flag flag
    int arm_disarm(bool flag);

    /// @brief Enable offboard control
    void enable_offboard_control();

    /// @brief Disable offboard control
    void disable_offboard_control();

    /// @brief Start
    void start();

    /// @brief Stop
    void stop();

    /// @brief Start read thread
    void start_read_thread();

    /// @brief Start write thread
    void start_write_thread();

    /// @brief Handle quit
    /// @param[in] sig Signal
    void handle_quit(int sig);

    /// @brief GPS input
    /// @param[in] lat_d Latitude
    /// @param[in] lon_d Longitude
    /// @param[in] alt Altitude
    /// @param[in] vn Velocity north
    /// @param[in] ve Velocity east
    /// @param[in] vd Velocity down
    /// @param[in] time_week_ms Time of week in [ms]
    /// @param[in] time_week Time of week
    int GPS_Input(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week);

    /// @brief MOCAP input
    /// @param[in] w scalar part
    /// @param[in] x vector component x
    /// @param[in] y vector component y
    /// @param[in] z vector component z
    int MOCAP_Input(float w, float x, float y, float z);

    /// @brief Set MOCAP
    /// @param[in] w scalar part
    /// @param[in] x vector component x
    /// @param[in] y vector component y
    /// @param[in] z vector component z
    void setMOCAP(float w, float x, float y, float z);

    /// @brief Set GPS
    /// @param[in] lat_d Latitude
    /// @param[in] lon_d Longitude
    /// @param[in] alt Altitude
    /// @param[in] vn Velocity north
    /// @param[in] ve Velocity east
    /// @param[in] vd Velocity down
    /// @param[in] time_week_ms Time of week in [ms]
    /// @param[in] time_week Time of week
    void setGPS(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week);

    /// @brief Set port
    /// @param[in] port_new Generic port
    void setPort(Generic_Port* port_new);

    /// @brief Set GPS frequency
    /// @param[in] _Frequency Frequency to set
    void setGPS_Frequency(double _Frequency);

    /// @brief Set MOCAP frequency
    /// @param[in] _Frequency Frequency to set
    void setMOCAP_Frequency(double _Frequency);

    /// @brief Set GPS active
    /// @param[in] _Active Flag that indicates whether GPS is active
    void setGPS_Active(bool _Active);

    /// @brief Set MOCAP active
    /// @param[in] _Active Flag that indicates whether MOCAP is active
    void setMOCAP_Active(bool _Active);

  private:
    Generic_Port* port = nullptr; ///< Generic port

    bool time_to_exit = false; ///< Time to exit

    pthread_t read_tid{};  ///< pthread read
    pthread_t write_tid{}; ///< pthread write

    /// @brief Current setpoint
    struct
    {
        std::mutex mutex;                               ///< mutex
        mavlink_set_position_target_local_ned_t data{}; ///< data
    } current_setpoint;

    /// @brief Read Thread
    void read_thread();

    /// @brief Write Thread
    void write_thread();

    /// @brief Toggle offboard control
    /// @param[in] flag Flag for the toggle
    int toggle_offboard_control(bool flag);

    /// @brief Write setpoint
    void write_setpoint();
};

#endif