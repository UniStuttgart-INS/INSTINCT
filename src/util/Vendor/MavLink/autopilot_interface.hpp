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
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION = 0b0000110111111000;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY = 0b0000110111000111;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION = 0b0000110000111111;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE = 0b0000111000111111;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE = 0b0000100111111111;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE = 0b0000010111111111;

constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF = 0x1000;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND = 0x2000;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER = 0x3000;
constexpr size_t MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE = 0x4000;

/// Available Baudrates
#define AUTOPILOT_MODE_STABILIZED 0   /// Self-levels the roll and pitch axis
#define AUTOPILOT_MODE_ACRO 1         /// Holds attitude, no self-level
#define AUTOPILOT_MODE_ALTHOLD 2      /// Holds altitude and self-levels the roll & pitch
#define AUTOPILOT_MODE_AUTO 3         /// Executes pre-defined mission
#define AUTOPILOT_MODE_GUIDED 4       /// Navigates to single points commanded by GCS
#define AUTOPILOT_MODE_LOITER 5       /// Holds altitude and position, uses GPS for movements
#define AUTOPILOT_MODE_RTL 6          /// Returns above takeoff location, may also include landing
#define AUTOPILOT_MODE_CIRCLE 7       /// Automatically circles a point in front of the vehicle
#define AUTOPILOT_MODE_LAND 9         /// Reduces altitude to ground level, attempts to go straight down
#define AUTOPILOT_MODE_DRIFT 11       /// Like stabilize, but coordinates yaw with roll like a plane
#define AUTOPILOT_MODE_SPORT 13       /// Alt-hold, but holds pitch & roll when sticks centered
#define AUTOPILOT_MODE_FLIP 14        /// Rises and completes an automated flip
#define AUTOPILOT_MODE_AUTOTUNE 15    /// Automated pitch and bank procedure to improve control loops
#define AUTOPILOT_MODE_POSHOLD 16     /// Like loiter, but manual roll and pitch when sticks not centered
#define AUTOPILOT_MODE_BRAKE 17       /// Brings copter to an immediate stop
#define AUTOPILOT_MODE_THROW 18       /// Holds position after a throwing takeoff
#define AUTOPILOT_MODE_AVOIDADSB 19   /// ADS-B based avoidance of manned aircraft
#define AUTOPILOT_MODE_GUIDEDNOGPS 20 /// This variation of Guided mode does not require a GPS but it only accepts attitude targets
#define AUTOPILOT_MODE_SMARTRTL 21    /// RTL, but traces path to get home
#define AUTOPILOT_MODE_FOLOWHOLD 22   /// Position control using Optical Flow
#define AUTOPILOT_MODE_FOLLOW 23      /// Follows another vehicle
#define AUTOPILOT_MODE_ZIGZAG 24      /// Useful for crop spraying
#define AUTOPILOT_MODE_SYSTEMID 25    /// Special diagnostic/modeling mode

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// helper functions
uint64_t
    get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t& sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t& sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t& sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t& sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t& sp);

void* start_autopilot_interface_read_thread(void* args);
void* start_autopilot_interface_write_thread(void* args);

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat{};
    uint64_t sys_status{};
    uint64_t battery_status{};
    uint64_t radio_status{};
    uint64_t local_position_ned{};
    uint64_t global_position_int{};
    uint64_t position_target_local_ned{};
    uint64_t position_target_global_int{};
    uint64_t highres_imu{};
    uint64_t attitude{};

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

// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages
{
    int sysid{};
    int compid{};

    // Heartbeat
    mavlink_heartbeat_t heartbeat{};

    // System Status
    mavlink_sys_status_t sys_status{};

    // Battery Status
    mavlink_battery_status_t battery_status{};

    // Radio Status
    mavlink_radio_status_t radio_status{};

    // Local Position
    mavlink_local_position_ned_t local_position_ned{};

    // Global Position
    mavlink_global_position_int_t global_position_int{};

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned{};

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int{};

    // HiRes IMU
    mavlink_highres_imu_t highres_imu{};

    // Attitude
    mavlink_attitude_t attitude{};

    // System Parameters?

    // Time Stamps
    Time_Stamps time_stamps{};

    void
        reset_timestamps()
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

    uint64_t write_count{};
    char reading_status{};
    char writing_status{};
    char control_status{};

    int system_id{};
    int autopilot_id{};
    int companion_id{};

    // MOCAP Variables
    float mocap_w{}, mocap_x{}, mocap_y{}, mocap_z{};
    bool MOCAP_Active = false;
    double MOCAP_Frequency = 0;

    // GPS Variables
    int32_t gps_lat_d{}, gps_lon_d{};
    float gps_alt{}, gps_vn{}, gps_ve{}, gps_vd{};
    uint32_t gps_time_week_ms{};
    uint16_t gps_time_week{};
    bool GPS_Active = false;
    double GPS_Frequency = 0;

    Mavlink_Messages current_messages{};
    mavlink_set_position_target_local_ned_t initial_position{};

    void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
    void read_messages();
    int write_message(mavlink_message_t message);

    int arm_disarm(bool flag);
    void enable_offboard_control();
    void disable_offboard_control();

    void start();
    void stop();

    void start_read_thread();
    void start_write_thread();

    void handle_quit(int sig);

    int GPS_Input(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week);
    int MOCAP_Input(float w, float x, float y, float z);
    void setMOCAP(float w, float x, float y, float z);
    void setGPS(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week);
    void setPort(Generic_Port* port_new);
    void setGPS_Frequency(double _Frequency);
    void setMOCAP_Frequency(double _Frequency);
    void setGPS_Active(bool _Active);
    void setMOCAP_Active(bool _Active);

  private:
    Generic_Port* port = nullptr;

    bool time_to_exit = false;

    pthread_t read_tid{};
    pthread_t write_tid{};

    struct
    {
        std::mutex mutex;
        mavlink_set_position_target_local_ned_t data{};
    } current_setpoint;

    void read_thread();
    void write_thread();

    int toggle_offboard_control(bool flag);
    void write_setpoint();
};
