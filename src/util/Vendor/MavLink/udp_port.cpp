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
 * @file udp_port.cpp
 *
 * @brief UDP interface functions
 *
 * Functions for opening, closing, reading and writing via UDP ports
 *
 * @author Hannes Diethelm, <hannes.diethelm@gmail.com>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "udp_port.hpp"
#include "util/Logger.hpp"

// ----------------------------------------------------------------------------------
//   UDP Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
UDP_Port::UDP_Port(const char* target_ip_, int udp_port_) : lastStatus(), lock(), buff(), buff_ptr(), buff_len(), debug(false), target_ip(target_ip_), rx_port(udp_port_), tx_port(-1), sock(-1), is_open(false)
{
    initialize_defaults();
}

UDP_Port::UDP_Port()
{
    initialize_defaults();
}

UDP_Port::~UDP_Port()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}

void UDP_Port::initialize_defaults()
{
    // Start mutex
    int result = pthread_mutex_init(&lock, nullptr);
    if (result != 0)
    {
        LOG_DEBUG("Mavlink - udp_port.cpp: mutex init failed");
    }
}

// ------------------------------------------------------------------------------
//   Read from UDP
// ------------------------------------------------------------------------------
int UDP_Port::read_message(mavlink_message_t& message)
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
            // printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            LOG_ERROR("Mavlink - udp_port.cpp: DROPPED {} PACKETS\n", status.packet_rx_drop_count);
            // unsigned char v = cp;
            // fprintf(stderr, "%02x ", v);
        }
        lastStatus = status;
    }

    // Couldn't read from port
    else
    {
        // fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n", result, errno);
        LOG_ERROR("Mavlink - udp_port.cpp: Could not read, res = {}, errno = {} \n", result, errno);
    }

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if (msgReceived && debug)
    {
        // Report info
        // printf("Received message from UDP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
        // LOG_INFO("Mavlink - udp_port.cpp: Received message from UDP with ID #{} (sys:{}|comp:{}):\n", message.msgid, message.sysid, message.compid); // FIXME: cannot bind bit-field 'message.__mavlink_message::msgid' to 'unsigned int&'

        LOG_INFO("Mavlink - udp_port.cpp: Received UDP data: ");
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};

        // check message is write length
        auto messageLength = mavlink_msg_to_send_buffer(buffer.data(), &message);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            // fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
            LOG_ERROR("Mavlink - udp_port.cpp: FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE");
        }

        // print out the buffer
        else
        {
            for (size_t i = 0; i < static_cast<size_t>(messageLength); i++)
            {
                [[maybe_unused]] unsigned char v = buffer.at(i);
                // fprintf(stderr, "%02x ", v);
                LOG_INFO("Mavlink - udp_port.cpp: {}", v);
            }
        }
    }

    // Done!
    return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to UDP
// ------------------------------------------------------------------------------
int UDP_Port::write_message(const mavlink_message_t& message)
{
    std::array<char, 300> buf{};

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buf.data()), &message);

    // Write buffer to UDP port, locks port while writing
    int bytesWritten = _write_port(buf.data(), len);
    if (bytesWritten < 0)
    {
        // fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n", bytesWritten, errno);
        LOG_ERROR("Mavlink - udp_port.cpp: Could not write, res = {}, errno = {}\n", bytesWritten, errno);
    }

    return bytesWritten;
}

// ------------------------------------------------------------------------------
//   Open UDP Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void UDP_Port::start()
{
    // --------------------------------------------------------------------------
    //   OPEN PORT
    // --------------------------------------------------------------------------

    /* Create socket */
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        LOG_ERROR("Mavlink - udp_port.cpp: error socket failed");
    }

    /* Bind the socket to rx_port - necessary to receive packets */
    struct sockaddr_in addr
    {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(target_ip);
    ;
    addr.sin_port = htons(static_cast<uint16_t>(rx_port));

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(struct sockaddr)))
    {
        LOG_ERROR("Mavlink - udp_port.cpp: error bind failed");
        close(sock);
        sock = -1;
    }
    /*if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        sock = -1;
        throw EXIT_FAILURE;
    }*/

    // --------------------------------------------------------------------------
    //   CONNECTED!
    // --------------------------------------------------------------------------
    // printf("Listening to %s:%i\n", target_ip, rx_port);
    LOG_INFO("Mavlink - udp_port.cpp: Listening to {}:{}\n", target_ip, rx_port);
    lastStatus.packet_rx_drop_count = 0;

    is_open = true;
}

// ------------------------------------------------------------------------------
//   Close UDP Port
// ------------------------------------------------------------------------------
void UDP_Port::stop()
{
    // printf("CLOSE PORT\n");
    LOG_INFO("Mavlink - udp_port.cpp: CLOSE PORT");

    int result = close(sock);
    sock = -1;

    if (result)
    {
        // fprintf(stderr, "WARNING: Error on port close (%i)\n", result);
        LOG_WARN("Mavlink - udp_port.cpp: Error on port close ({})\n", result);
    }

    is_open = false;

    // printf("\n");
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_read_port(uint8_t& cp)
{
    socklen_t len{};

    // Lock
    pthread_mutex_lock(&lock);

    int result = -1;
    if (buff_ptr < buff_len)
    {
        cp = buff.at(static_cast<uint8_t>(buff_ptr));
        buff_ptr++;
        result = 1;
    }
    else
    {
        struct sockaddr_in addr
        {};
        len = sizeof(struct sockaddr_in);
        result = static_cast<int>(recvfrom(sock, &buff, BUFF_LEN, 0, reinterpret_cast<struct sockaddr*>(&addr), &len));
        if (tx_port < 0)
        {
            if (strcmp(inet_ntoa(addr.sin_addr), target_ip) == 0) // NOLINT(concurrency-mt-unsafe)
            {
                tx_port = ntohs(addr.sin_port);
                // printf("Got first packet, sending to %s:%i\n", target_ip, rx_port);
                LOG_INFO("Mavlink - udp_port.cpp: Got first packet, sending to {}:{}\n", target_ip, rx_port);
            }
            else
            {
                // printf("ERROR: Got packet from %s:%i but listening on %s\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port), target_ip);
                LOG_ERROR("Mavlink - udp_port.cpp: Got packet from {}:{} but listening on {}\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port), target_ip); // NOLINT(concurrency-mt-unsafe)
            }
        }
        if (result > 0)
        {
            buff_len = result;
            buff_ptr = 0;
            cp = buff.at(static_cast<uint8_t>(buff_ptr));
            buff_ptr++;
            // printf("recvfrom: %i %i\n", result, cp);
        }
    }

    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}

// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_write_port(char* buf, unsigned len)
{
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via UDP link
    int bytesWritten = 0;
    if (tx_port > 0)
    {
        struct sockaddr_in addr
        {};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(target_ip);
        addr.sin_port = htons(static_cast<uint16_t>(tx_port));
        bytesWritten = static_cast<int8_t>(sendto(sock, buf, len, 0, reinterpret_cast<struct sockaddr*>(&addr), sizeof(struct sockaddr_in))); // NOLINT(bugprone-signed-char-misuse)
        // printf("sendto: %i\n", bytesWritten);
    }
    else
    {
        // printf("ERROR: Sending before first packet received!\n");
        LOG_ERROR("Mavlink - udp_port.cpp: Sending before first packet received!\n");
        bytesWritten = -1;
    }

    // Unlock
    pthread_mutex_unlock(&lock);

    return bytesWritten;
}
