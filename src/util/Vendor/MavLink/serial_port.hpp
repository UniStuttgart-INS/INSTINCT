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
 * @file serial_port.hpp
 *
 * @brief Serial interface definition
 *
 * Functions for opening, closing, reading and writing via serial ports
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

    #include <cstdlib>
    #include <unistd.h>  // UNIX standard function definitions
    #include <fcntl.h>   // File control definitions
    #include <termios.h> // POSIX terminal control definitions
    #include <pthread.h> // This uses POSIX Threads

    #include <mavlink/common/mavlink.h>

    #include "generic_port.hpp"

    // ------------------------------------------------------------------------------
    //   Defines
    // ------------------------------------------------------------------------------

    // The following two non-standard baudrates should have been defined by the system
    // If not, just fallback to number
    #ifndef B460800
        #define B460800 460800 ///< Baudrate
    #endif

    #ifndef B921600
        #define B921600 921600 ///< Baudrate
    #endif

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// class Serial_Port;

// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */

/// @brief Serial Port Class
class Serial_Port : public Generic_Port
{
  public:
    /// @brief Default constructor
    Serial_Port();
    /// @brief Constructor
    Serial_Port(const char* uart_name_, int baudrate_);
    /// @brief Destructor
    ~Serial_Port() override;
    /// @brief Copy constructor
    Serial_Port(const Serial_Port&) = delete;
    /// @brief Move constructor
    Serial_Port(Serial_Port&&) = delete;
    /// @brief Copy assignment operator
    Serial_Port& operator=(const Serial_Port&) = delete;
    /// @brief Move assignment operator
    Serial_Port& operator=(Serial_Port&&) = delete;

    int read_message(mavlink_message_t& message) override;
    int write_message(const mavlink_message_t& message) override;

    /// @brief is running
    bool is_running() override
    {
        return is_open;
    }

    /// @brief Start
    void start() override;

    /// @brief Stop
    void stop() override;

  private:
    int fd;                      ///< fd?
    mavlink_status_t lastStatus; ///< Last Mavlink status
    pthread_mutex_t lock;        ///< pthread mutex lock

    /// @brief Initialize defaults
    void initialize_defaults();

    bool debug;            ///< debug flag
    const char* uart_name; ///< uart name
    int baudrate;          ///< Baudrate
    bool is_open;          ///< Is open

    /// @brief Open port
    /// @param[in] port Port to open
    int _open_port(const char* port);

    /// @brief Setup of port
    /// @param[in] baud Baud rate
    /// @param[in] data_bits Data bits
    /// @param[in] stop_bits Stop bits
    /// @param[in] parity Parity
    /// @param[in] hardware_control Hardware control
    [[nodiscard]] bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) const;

    /// @brief Read port
    /// @param[in] cp Port to read
    int _read_port(uint8_t& cp);

    /// @brief Write port
    /// @param[in] buf Buffer to write
    /// @param[in] len Length of buffer to write
    int _write_port(char* buf, unsigned len);
};

#endif