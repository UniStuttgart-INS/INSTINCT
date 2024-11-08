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
 * @file serial_port.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#if __linux__ || __APPLE__
    #include "serial_port.hpp"
    #include "util/Logger.hpp"

// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial_Port::Serial_Port(const char* uart_name_, int baudrate_) : fd(-1), lastStatus(), lock(), debug(false), uart_name(uart_name_), baudrate(baudrate_), is_open(false)
{
    initialize_defaults();
}

Serial_Port::Serial_Port() : fd(-1), lastStatus(), lock(), debug(false), uart_name("/dev/ttyUSB0"), baudrate(57600), is_open(false)
{
    initialize_defaults();
}

Serial_Port::~Serial_Port()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}

void Serial_Port::initialize_defaults()
{
    // Initialize attributes
    debug = false;
    fd = -1;
    is_open = false;

    uart_name = "/dev/ttyUSB0";
    baudrate = 57600;

    // Start mutex
    int result = pthread_mutex_init(&lock, nullptr);
    if (result != 0)
    {
        LOG_ERROR("serial_port.cpp - mutex init failed\n");
    }
}

// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int Serial_Port::read_message(mavlink_message_t& message)
{
    uint8_t cp{};
    mavlink_status_t status;
    uint8_t msgReceived = false;

    // --------------------------------------------------------------------------
    //   READ FROM PORT
    // --------------------------------------------------------------------------

    // this function locks the port during read
    int result = _read_port(cp);

    // --------------------------------------------------------------------------
    //   PARSE MESSAGE
    // --------------------------------------------------------------------------
    if (result > 0)
    {
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        // check for dropped packets
        if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
        {
            LOG_ERROR("serial_port.cpp - DROPPED {} PACKETS\n", status.packet_rx_drop_count);
            [[maybe_unused]] unsigned char v = cp;
            LOG_ERROR("serial_port.cpp - {}", v);
        }
        lastStatus = status;
    }

    // Couldn't read from port
    else
    {
        cabelCheck = true;
        stop();
        LOG_ERROR("serial_port.cpp - Could not read from fd {}\n", fd);
    }

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if (msgReceived && debug)
    {
        // Report info
        // LOG_INFO("serial_port.cpp - Received message from serial with ID #{} (sys:{}|comp:{}):\n", message.msgid, message.sysid, message.compid);

        LOG_ERROR("serial_port.cpp - Received serial data");
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};

        // check message is write length
        auto messageLength = mavlink_msg_to_send_buffer(buffer.data(), &message);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            LOG_ERROR("serial_port.cpp - MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }

        // print out the buffer
        else
        {
            for (size_t i = 0; i < messageLength; i++)
            {
                [[maybe_unused]] auto v = buffer.at(i);
                LOG_ERROR("serial_port.cpp - {}", v);
            }
        }
    }

    // Done!
    return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int Serial_Port::write_message(const mavlink_message_t& message)
{
    std::array<char, 300> buf{};

    // Translate message to buffer
    auto len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buf.data()), &message);

    // Write buffer to serial port, locks port while writing
    auto bytesWritten = _write_port(buf.data(), len);

    return bytesWritten;
}

// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void Serial_Port::start()
{
    // --------------------------------------------------------------------------
    //   OPEN PORT
    // --------------------------------------------------------------------------
    LOG_INFO("serial_port.cpp - OPEN PORT\n");

    fd = _open_port(uart_name);

    // Check success
    if (fd == -1)
    {
        LOG_ERROR("serial_port.cpp - failure, could not open port.\n");
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   SETUP PORT
    // --------------------------------------------------------------------------
    auto success = _setup_port(baudrate, 8, 1, false, false);

    // --------------------------------------------------------------------------
    //   CHECK STATUS
    // --------------------------------------------------------------------------
    if (!success)
    {
        LOG_ERROR("serial_port.cpp - failure, could not configure port.\n");
        throw EXIT_FAILURE;
    }
    if (fd <= 0)
    {
        LOG_ERROR("serial_port.cpp - Connection attempt to port {} with {} baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   CONNECTED!
    // --------------------------------------------------------------------------
    LOG_INFO("serial_port.cpp - Connected to {} with {} baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    lastStatus.packet_rx_drop_count = 0;

    is_open = true;
}

// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void Serial_Port::stop()
{
    LOG_INFO("serial_port.cpp - CLOSE PORT\n");

    int result = close(fd);

    if (result)
    {
        LOG_WARN("serial_port.cpp - Error on port close ({})\n", result);
    }

    is_open = false;
}

// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int Serial_Port::_open_port(const char* port)
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY | O_CLOEXEC);

    // Check for Errors
    if (fd == -1)
    {
        LOG_ERROR("serial_port.cpp - Could not open the port.");
    }
    // Finalize
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool Serial_Port::_setup_port(int baud, int /* data_bits */, int /* stop_bits */, bool /* parity */, bool /* hardware_control */)
{
    // Check file descriptor
    if (!isatty(fd))
    {
        LOG_ERROR("serial_port.cpp - file descriptor {} is NOT a serial port\n", fd);
        return false;
    }

    // Read file descritor configuration
    struct termios config
    {};

    if (tcgetattr(fd, &config) < 0)
    {
        LOG_ERROR("serial_port.cpp - could not read configuration of fd {}\n", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~static_cast<tcflag_t>(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~static_cast<tcflag_t>(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    #ifdef OLCUC
    config.c_oflag &= ~static_cast<tcflag_t>(OLCUC);
    #endif

    #ifdef ONOEOT
    config.c_oflag &= ~static_cast<tcflag_t>(ONOEOT);
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~static_cast<tcflag_t>(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud)
    {
    case 1200:
        if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    case 1800:
        cfsetispeed(&config, B1800);
        cfsetospeed(&config, B1800);
        break;
    case 9600:
        cfsetispeed(&config, B9600);
        cfsetospeed(&config, B9600);
        break;
    case 19200:
        cfsetispeed(&config, B19200);
        cfsetospeed(&config, B19200);
        break;
    case 38400:
        if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    case 57600:
        if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    case 115200:
        if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;

    // These two non-standard (by the 70'ties ) rates are fully supported on
    // current Debian and Mac OS versions (tested since 2010).
    case 460800:
        if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    case 921600:
        if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    #ifndef __APPLE__
    case 1500000:
        if (cfsetispeed(&config, B1500000) < 0 || cfsetospeed(&config, B1500000) < 0)
        {
            LOG_ERROR("serial_port.cpp - Could not set desired baud rate of {} Baud\n", baud);
            return false;
        }
        break;
    #endif
    default:
        LOG_ERROR("serial_port.cpp - Desired baud rate {} could not be set, aborting.\n", baud);
        return false;

        break;
    }

    // Finally, apply the configuration
    if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        LOG_ERROR("serial_port.cpp - could not set configuration of fd {}\n", fd);
        return false;
    }

    // Done!
    return true;
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int Serial_Port::_read_port(uint8_t& cp)
{
    // Lock
    pthread_mutex_lock(&lock);

    auto result = static_cast<int8_t>(read(fd, &cp, 1)); // NOLINT(bugprone-signed-char-misuse,cert-str34-c)

    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}

// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int Serial_Port::_write_port(char* buf, unsigned len)
{
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd, buf, len));

    // Wait until all data has been written
    tcdrain(fd);

    // Unlock
    pthread_mutex_unlock(&lock);

    return bytesWritten;
}

#endif