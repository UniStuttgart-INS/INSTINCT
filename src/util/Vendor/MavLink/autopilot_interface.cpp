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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <optional>

#include "autopilot_interface.hpp"
#include "util/Logger.hpp"

// ----------------------------------------------------------------------------------
//   Time
// ----------------------------------------------------------------------------------
uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, nullptr);
    return static_cast<uint64_t>(_time_stamp.tv_sec) * 1000000UL + static_cast<uint64_t>(_time_stamp.tv_usec);
}

// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t& sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    LOG_INFO("autopilot_interface.cpp - POSITION SETPOINT XYZ = [ {} , {} , {} ] \n", sp.x, sp.y, sp.z);
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t& sp)
{
    sp.type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    LOG_INFO("autopilot_interface.cpp - VELOCITY SETPOINT UVW = [ {} , {} , {} ] \n", sp.vx, sp.vy, sp.vz);
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float /* ax */, float /* ay */, float /* az */, mavlink_set_position_target_local_ned_t& /* sp */)
{
    // NOT IMPLEMENTED
    LOG_ERROR("autopilot_interface.cpp - set_acceleration doesn't work yet \n");

    // sp.type_mask =
    //     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    // sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    // sp.afx = ax;
    // sp.afy = ay;
    // sp.afz = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t& sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    LOG_INFO("autopilot_interface.cpp - POSITION SETPOINT YAW = {} \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t& sp)
{
    sp.type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::Autopilot_Interface(Generic_Port* port_) : port(port_)
{
    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;
}

// ------------------------------------------------------------------------------
//   Set Port
// ------------------------------------------------------------------------------
void Autopilot_Interface::setPort(Generic_Port* port_new)
{
    port = port_new;
}

// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    std::lock_guard<std::mutex> lock(current_setpoint.mutex);
    current_setpoint.data = setpoint;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_messages()
{
    bool success = false;      // receive success flag
    bool received_all = false; // receive only one message
    Time_Stamps this_timestamps;

    // Blocking wait for new data
    while (!received_all and !time_to_exit)
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        success = port->read_message(message);

        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if (success)
        {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_HEARTBEAT\n");
                mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                current_messages.time_stamps.heartbeat = get_time_usec();
                this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                break;
            }

            case MAVLINK_MSG_ID_SYS_STATUS:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_SYS_STATUS\n");
                mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                current_messages.time_stamps.sys_status = get_time_usec();
                this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                break;
            }

            case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_BATTERY_STATUS\n");
                mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                current_messages.time_stamps.battery_status = get_time_usec();
                this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                break;
            }

            case MAVLINK_MSG_ID_RADIO_STATUS:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_RADIO_STATUS\n");
                mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                current_messages.time_stamps.radio_status = get_time_usec();
                this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                break;
            }

            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                current_messages.time_stamps.local_position_ned = get_time_usec();
                this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
                break;
            }

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                current_messages.time_stamps.global_position_int = get_time_usec();
                this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                current_messages.time_stamps.position_target_local_ned = get_time_usec();
                this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                break;
            }

            case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                current_messages.time_stamps.position_target_global_int = get_time_usec();
                this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
                break;
            }

            case MAVLINK_MSG_ID_HIGHRES_IMU:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_HIGHRES_IMU\n");
                mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                current_messages.time_stamps.highres_imu = get_time_usec();
                this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                break;
            }

            case MAVLINK_MSG_ID_ATTITUDE:
            {
                // LOG_INFO("autopilot_interface.cpp - MAVLINK_MSG_ID_ATTITUDE\n");
                mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                current_messages.time_stamps.attitude = get_time_usec();
                this_timestamps.attitude = current_messages.time_stamps.attitude;
                break;
            }

            default:
            {
                // LOG_WARN("Warning, did not handle message id %i\n",message.msgid);
                break;
            }

            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        received_all =
            this_timestamps.heartbeat &&
            //				this_timestamps.battery_status             &&
            //				this_timestamps.radio_status               &&
            //				this_timestamps.local_position_ned         &&
            //				this_timestamps.global_position_int        &&
            //				this_timestamps.position_target_local_ned  &&
            //				this_timestamps.position_target_global_int &&
            //				this_timestamps.highres_imu                &&
            //				this_timestamps.attitude                   &&
            this_timestamps.sys_status;

        // give the write thread time to use the port
        if (writing_status > false)
        {
            usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Autopilot_Interface::write_message(mavlink_message_t message)
{
    // do the write
    int len = port->write_message(message);

    // book keep
    write_count++;

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // pull from position target
    mavlink_set_position_target_local_ned_t sp;
    {
        std::lock_guard<std::mutex> lock(current_setpoint.mutex);
        sp = current_setpoint.data;
    }

    // double check some system parameters

    if (!sp.time_boot_ms)
    {
        sp.time_boot_ms = static_cast<uint32_t>(get_time_usec() / 1000);
    }
    sp.target_system = static_cast<uint8_t>(system_id);
    sp.target_component = static_cast<uint8_t>(autopilot_id);

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(static_cast<uint8_t>(system_id), static_cast<uint8_t>(companion_id), &message, &sp);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if (len <= 0)
    {
        LOG_WARN("autopilot_interface.cpp - could not send POSITION_TARGET_LOCAL_NED \n");
    }
    //	else
    //		LOG_INFO("autopilot_interface.cpp - {} POSITION_TARGET  = [ {} , {} , {} ] \n", write_count, position_target.x, position_target.y, position_target.z);
}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void Autopilot_Interface::enable_offboard_control()
{
    // Should only send this command once
    if (!static_cast<bool>(control_status))
    {
        LOG_INFO("autopilot_interface.cpp - ENABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to go off-board
        int success = toggle_offboard_control(true);

        // Check the command was written
        if (success)
        {
            control_status = true;
        }
        else
        {
            LOG_ERROR("autopilot_interface.cpp - off-board mode not set, could not write message\n");
            return;
        }

    } // end: if not offboard_status
}

// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void Autopilot_Interface::disable_offboard_control()
{
    // Should only send this command once
    if (static_cast<bool>(control_status))
    {
        LOG_INFO("autopilot_interface.cpp - DISABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to stop off-board
        int success = toggle_offboard_control(false);

        // Check the command was written
        if (success)
        {
            control_status = false;
        }
        else
        {
            LOG_ERROR("autopilot_interface.cpp - off-board mode not set, could not write message\n");
            return;
        }

    } // end: if offboard_status
}

// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
int Autopilot_Interface::arm_disarm(bool flag)
{
    if (flag)
    {
        LOG_INFO("autopilot_interface.cpp - ARM ROTORS\n");
    }
    else
    {
        LOG_INFO("autopilot_interface.cpp - DISARM ROTORS\n");
    }

    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0, NAN, NAN, NAN, NAN, NAN, NAN, 0, 0, 0, 0 }; // FIXME: init values could be wrong here, but have to be set
    com.target_system = static_cast<uint8_t>(system_id);
    com.target_component = static_cast<uint8_t>(autopilot_id);
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = true;
    com.param1 = static_cast<float>(flag);
    com.param2 = 21196.;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(static_cast<uint8_t>(system_id), static_cast<uint8_t>(companion_id), &message, &com);

    // Send the message
    int len = port->write_message(message);

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int Autopilot_Interface::toggle_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = { 0, NAN, NAN, NAN, NAN, NAN, NAN, 0, 0, 0, 0 }; // FIXME: init values could be wrong here, but have to be set
    com.target_system = static_cast<uint8_t>(system_id);
    com.target_component = static_cast<uint8_t>(autopilot_id);
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = static_cast<float>(flag); // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(static_cast<uint8_t>(system_id), static_cast<uint8_t>(companion_id), &message, &com);

    // Send the message
    int len = port->write_message(message);

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   MOCAP_Input (ATT_POS_MOCAP (#138))
// ------------------------------------------------------------------------------
int Autopilot_Interface::MOCAP_Input(float w, float x, float y, float z)
{
    mavlink_att_pos_mocap_t current_MOCAP{};

    current_MOCAP.time_usec = get_time_usec();
    current_MOCAP.q[0] = w; // Attitude quaternion W
    current_MOCAP.q[1] = x; // Attitude quaternion X
    current_MOCAP.q[2] = y; // Attitude quaternion Y
    current_MOCAP.q[3] = z; // Attitude quaternion Z
    current_MOCAP.x = 0;    // X position (NED) (m)
    current_MOCAP.y = 0;    // Y position (NED) (m)
    current_MOCAP.z = 0;    // Z position (NED) (m)

    current_MOCAP.covariance[0] = { std::nanf("1") };
    /*
    current_MOCAP.covariance[1] = 0;
    current_MOCAP.covariance[2] = 0;
    current_MOCAP.covariance[3] = 0;
    current_MOCAP.covariance[4] = 0;
    current_MOCAP.covariance[5] = 0;

    current_MOCAP.covariance[6] = 0;
    current_MOCAP.covariance[7] = 0;
    current_MOCAP.covariance[8] = 0;
    current_MOCAP.covariance[9] = 0;
    current_MOCAP.covariance[10] = 0;
    current_MOCAP.covariance[11] = 0;
    current_MOCAP.covariance[12] = 0;
    current_MOCAP.covariance[13] = 0;
    current_MOCAP.covariance[14] = 0;
    current_MOCAP.covariance[15] = 0;
    current_MOCAP.covariance[16] = 0;
    current_MOCAP.covariance[17] = 0;
    current_MOCAP.covariance[18] = 0;
    current_MOCAP.covariance[19] = 0;
    current_MOCAP.covariance[20] = 0;
    */

    // Encode
    mavlink_message_t message;
    mavlink_msg_att_pos_mocap_encode(static_cast<uint8_t>(system_id), static_cast<uint8_t>(companion_id), &message, &current_MOCAP); // NOLINT(clang-analyzer-core.CallAndMessage)

    // Send the message
    int len = port->write_message(message);

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   GPS_Input (GPS_INPUT (#232))
// ------------------------------------------------------------------------------

int Autopilot_Interface::GPS_Input(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week)
{
    mavlink_gps_input_t GPS;
    GPS.time_usec = get_time_usec();
    GPS.gps_id = 0;
    GPS.ignore_flags = GPS_INPUT_IGNORE_FLAG_HDOP | GPS_INPUT_IGNORE_FLAG_VDOP |
                       // GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                       // GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                       GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY | GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY | GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
    GPS.time_week_ms = time_week_ms;
    GPS.time_week = time_week;
    GPS.fix_type = 3;
    GPS.lat = lat_d; // Latitude (WGS84)  (degE7)
    GPS.lon = lon_d; // Longitude (WGS84) (degE7)
    GPS.alt = alt;   // Altitude (MSL)  Positive for up (m)
    // GPS.hdop = 0;
    // GPS.vdop = 0;
    GPS.vn = vn;
    GPS.ve = ve;
    GPS.vd = vd;
    // GPS.speed_accuracy = 0;
    // GPS.horiz_accuracy = 0;
    // GPS.vert_accuracy = 0;
    GPS.satellites_visible = 20;

    // Encode
    mavlink_message_t message;
    mavlink_msg_gps_input_encode(static_cast<uint8_t>(system_id), static_cast<uint8_t>(companion_id), &message, &GPS);

    // Send the message
    int len = port->write_message(message);

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   setGPS
// ------------------------------------------------------------------------------
void Autopilot_Interface::setGPS(int32_t lat_d, int32_t lon_d, float alt, float vn, float ve, float vd, uint32_t time_week_ms, uint16_t time_week)
{
    gps_lat_d = lat_d;
    gps_lon_d = lon_d;
    gps_alt = alt;
    gps_vn = vn;
    gps_ve = ve;
    gps_vd = vd;
    gps_time_week_ms = time_week_ms;
    gps_time_week = time_week;
}

// ------------------------------------------------------------------------------
//   setMOCAP
// ------------------------------------------------------------------------------
void Autopilot_Interface::setMOCAP(float w, float x, float y, float z)
{
    mocap_w = w;
    mocap_x = x;
    mocap_y = y;
    mocap_z = z;
}

// ------------------------------------------------------------------------------
//   setGPS_Active
// ------------------------------------------------------------------------------
void Autopilot_Interface::setGPS_Active(bool _Active)
{
    GPS_Active = _Active;
}

// ------------------------------------------------------------------------------
//   setMOCAP_Active
// ------------------------------------------------------------------------------
void Autopilot_Interface::setMOCAP_Active(bool _Active)
{
    MOCAP_Active = _Active;
}

// ------------------------------------------------------------------------------
//   setMOCAP_Frequency
// ------------------------------------------------------------------------------
void Autopilot_Interface::setMOCAP_Frequency(double _Frequency)
{
    MOCAP_Frequency = _Frequency;
}

// ------------------------------------------------------------------------------
//   setGPS_Frequency
// ------------------------------------------------------------------------------
void Autopilot_Interface::setGPS_Frequency(double _Frequency)
{
    GPS_Frequency = _Frequency;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::start()
{
    std::optional<int> result;
    time_to_exit = false;
    current_messages = {};
    // --------------------------------------------------------------------------
    //   CHECK PORT
    // --------------------------------------------------------------------------

    if (!port->is_running()) // PORT_OPEN
    {
        LOG_ERROR("autopilot_interface.cpp - port not open\n");
        time_to_exit = true;
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    LOG_INFO("autopilot_interface.cpp - START READ THREAD \n");

    result = pthread_create(&read_tid, nullptr, &start_autopilot_interface_read_thread, this);
    if (result.has_value())
    {
        LOG_INFO("result = {}", result);
    }

    // now we're reading messages

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

    LOG_INFO("autopilot_interface.cpp - CHECK FOR MESSAGES\n");
    int i = 0;
    while (!current_messages.sysid)
    {
        if (time_to_exit || i >= 20)
        {
            LOG_ERROR("autopilot_interface.cpp - NO MESSAGES FOUND\n");
            time_to_exit = true;
            throw EXIT_FAILURE;
        }
        usleep(500000); // check at 2Hz
        i++;            // wait only 10s
    }

    LOG_INFO("autopilot_interface.cpp - Found\n");

    // now we know autopilot is sending messages

    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if (!system_id)
    {
        system_id = current_messages.sysid;
        LOG_INFO("autopilot_interface.cpp - GOT VEHICLE SYSTEM ID: {}\n", system_id);
    }

    // Component ID
    if (!autopilot_id)
    {
        autopilot_id = current_messages.compid;
        LOG_INFO("autopilot_interface.cpp - GOT AUTOPILOT COMPONENT ID: {}\n", autopilot_id);
    }

    // --------------------------------------------------------------------------
    //   GET INITIAL POSITION
    // --------------------------------------------------------------------------

    // Wait for initial position ned
    /*
    while (not(current_messages.time_stamps.local_position_ned && current_messages.time_stamps.attitude))
    {
        if (time_to_exit)
        {
            return;
        }
        usleep(500000);
    }
*/
    // copy initial position ned
    Mavlink_Messages local_data = current_messages;
    initial_position.x = local_data.local_position_ned.x;
    initial_position.y = local_data.local_position_ned.y;
    initial_position.z = local_data.local_position_ned.z;
    initial_position.vx = local_data.local_position_ned.vx;
    initial_position.vy = local_data.local_position_ned.vy;
    initial_position.vz = local_data.local_position_ned.vz;
    initial_position.yaw = local_data.attitude.yaw;
    initial_position.yaw_rate = local_data.attitude.yawspeed;

    LOG_INFO("autopilot_interface.cpp - INITIAL POSITION XYZ = [ {} , {} , {} ] \n", initial_position.x, initial_position.y, initial_position.z);
    LOG_INFO("autopilot_interface.cpp - INITIAL POSITION YAW = {} \n", initial_position.yaw);

    // we need this before starting the write thread

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    LOG_INFO("autopilot_interface.cpp - START WRITE THREAD \n");

    result = pthread_create(&write_tid, nullptr, &start_autopilot_interface_write_thread, this);
    if (result.has_value())
    {
        LOG_INFO("result = {}", result);
    }

    // wait for it to be started
    while (!writing_status)
    {
        usleep(100000); // 10Hz
    }

    // now we're streaming setpoint commands

    // Done!
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void Autopilot_Interface::stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    LOG_INFO("autopilot_interface.cpp - CLOSE THREADS\n");

    // signal exit
    time_to_exit = true;

    // wait for exit
    pthread_join(read_tid, nullptr);
    pthread_join(write_tid, nullptr);

    // now the read and write threads are closed

    // still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_read_thread()
{
    if (reading_status != 0)
    {
        LOG_WARN("autopilot_interface.cpp - read thread already running\n");
    }
    else
    {
        read_thread();
    }
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_write_thread()
{
    if (static_cast<bool>(writing_status))
    {
        LOG_ERROR("autopilot_interface.cpp - write thread already running\n");
    }
    else
    {
        write_thread();
    }
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void Autopilot_Interface::handle_quit(int /* sig */)
{
    disable_offboard_control();

    try
    {
        stop();
    }
    catch (const std::exception& /* e */)
    {
        LOG_ERROR("autopilot_interface.cpp - could not stop autopilot interface\n");
    }
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_thread()
{
    reading_status = true;

    while (!time_to_exit)
    {
        read_messages();
        usleep(100000); // Read batches at 10Hz
    }

    reading_status = false;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_thread()
{
    // signal startup
    writing_status = 2;
    /*
        // prepare an initial setpoint, just stay put
        mavlink_set_position_target_local_ned_t sp;
        sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
        sp.vx = 0.0;
        sp.vy = 0.0;
        sp.vz = 0.0;
        sp.yaw_rate = 0.0;

        // set position target
        {
            std::lock_guard<std::mutex> lock(current_setpoint.mutex);
            current_setpoint.data = sp;
        }
    */
    // write a message and signal writing
    // write_setpoint();
    writing_status = true;
    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe

    auto lastCallTimeGPS = std::chrono::system_clock::now();
    auto lastCallTimeMOCAP = std::chrono::system_clock::now();
    while (!time_to_exit)
    {
        auto currentTime = std::chrono::system_clock::now();
        auto elapsedTimeGPS = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastCallTimeGPS).count();
        auto elapsedTimeMOCAP = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastCallTimeMOCAP).count();

        if ((GPS_Frequency > 0.0) && (elapsedTimeGPS >= 1.0 / GPS_Frequency))
        {
            if (GPS_Active)
            {
                GPS_Input(gps_lat_d, gps_lon_d, gps_alt, gps_vn, gps_ve, gps_vd, gps_time_week_ms, gps_time_week);
            }
            lastCallTimeGPS = currentTime;
        }
        if ((MOCAP_Frequency > 0.0) && (elapsedTimeMOCAP >= 1.0 / MOCAP_Frequency))
        {
            if (MOCAP_Active)
            {
                MOCAP_Input(mocap_w, mocap_x, mocap_y, mocap_z);
            }
            lastCallTimeMOCAP = currentTime;
        }
        // usleep(100);
    }
    // signal end
    writing_status = false;
}

// End Autopilot_Interface

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void* start_autopilot_interface_read_thread(void* args)
{
    // takes an autopilot object argument
    auto* autopilot_interface = static_cast<Autopilot_Interface*>(args);

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return nullptr;
}

void* start_autopilot_interface_write_thread(void* args)
{
    // takes an autopilot object argument
    auto* autopilot_interface = static_cast<Autopilot_Interface*>(args);

    // run the object's read thread
    autopilot_interface->start_write_thread(); // NOLINT(clang-analyzer-core.CallAndMessage)

    // done!
    return nullptr;
}
