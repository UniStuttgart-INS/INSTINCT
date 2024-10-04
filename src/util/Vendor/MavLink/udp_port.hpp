/****************************************************************************
 *
 *   Copyright (c) 2018 MAVlink Development Team. All rights reserved.
 *   Author: Hannes Diethelm, <hannes.diethelm@gmail.com>
 *           Trent Lukaczyk, <aerialhedgehog@gmail.com>
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
 * @file udp_port.hpp
 *
 * @brief UDP interface definition
 *
 * Functions for opening, closing, reading and writing via UDP ports
 *
 * @author Hannes Diethelm, <hannes.diethelm@gmail.com>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

#pragma once

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <array>
#include <cstdlib>
#include <pthread.h> // This uses POSIX Threads
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include <mavlink/common/mavlink.h>

#include "generic_port.hpp"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
//   UDP Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * UDP Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * UDP port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class UDP_Port : public Generic_Port
{
  public:
    /// @brief Default constructor
    UDP_Port();
    /// @brief Constructor
    UDP_Port(const char* target_ip_, int udp_port_);
    /// @brief Destructor
    ~UDP_Port() override;
    /// @brief Copy constructor
    UDP_Port(const UDP_Port&) = delete;
    /// @brief Move constructor
    UDP_Port(UDP_Port&&) = delete;
    /// @brief Copy assignment operator
    UDP_Port& operator=(const UDP_Port&) = delete;
    /// @brief Move assignment operator
    UDP_Port& operator=(UDP_Port&&) = delete;

    int read_message(mavlink_message_t& message) override;
    int write_message(const mavlink_message_t& message) override;

    bool is_running() override
    {
        return is_open;
    }
    void start() override;
    void stop() override;

  private:
    mavlink_status_t lastStatus{};
    pthread_mutex_t lock{};

    void initialize_defaults();

    const static int BUFF_LEN = 2041;
    std::array<uint8_t, BUFF_LEN> buff{};
    int buff_ptr{};
    int buff_len{};
    bool debug = false;
    const char* target_ip = "127.0.0.1";
    int rx_port{ 14550 };
    int tx_port{ -1 };
    int sock{ -1 };
    bool is_open = false;

    int _read_port(uint8_t& cp);
    int _write_port(char* buf, unsigned len);
};
