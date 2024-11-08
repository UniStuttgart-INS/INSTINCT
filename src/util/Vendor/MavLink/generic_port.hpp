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
 * @file generic_port.hpp
 *
 * @brief Generic interface definition
 *
 * Abstract port definition
 *
 * @author Hannes Diethelm, <hannes.diethelm@gmail.com>
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

    #include <mavlink/common/mavlink.h>
// #include <common/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
//   Generic Port Manager Class
// ----------------------------------------------------------------------------------

/// @brief Generic Port Class (This is an abstract port definition to handle both serial and UDP ports)
class Generic_Port
{
  public:
    /// @brief Variable to de-initialize node if cable is pulled
    bool cabelCheck = false;
    /// @brief Default constructor
    Generic_Port() = default;
    /// @brief Destructor
    virtual ~Generic_Port() = default;
    /// @brief Copy constructor
    Generic_Port(const Generic_Port&) = delete;
    /// @brief Move constructor
    Generic_Port(Generic_Port&&) = delete;
    /// @brief Copy assignment operator
    Generic_Port& operator=(const Generic_Port&) = delete;
    /// @brief Move assignment operator
    Generic_Port& operator=(Generic_Port&&) = delete;

    /// @brief Read Mavlink message
    /// @param[in] message Mavlink message to read
    virtual int read_message(mavlink_message_t& message) = 0;

    /// @brief Write Mavlink message
    /// @param[in] message Mavlink message to write
    virtual int write_message(const mavlink_message_t& message) = 0;

    /// @brief Is running
    virtual bool is_running() = 0;

    /// @brief Start
    virtual void start() = 0;

    /// @brief Stop
    virtual void stop() = 0;
};

#endif