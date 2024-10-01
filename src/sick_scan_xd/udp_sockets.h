/*
 * @brief udp_sockets implement udp sender and receiver
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */

#pragma once

#if defined WIN32 || defined _MSC_VER
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#include <winsock2.h>
#define UNLINK _unlink
static std::string getErrorMessage(void)
{
    int error_num = WSAGetLastError();
    char error_message[1024] = { 0 };
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, error_num, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), error_message, sizeof(error_message), NULL);
    return std::to_string(error_num) + " (" + std::string(error_message) + ")";
}
#else
#include <cerrno>
#include <string>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <sys/types.h>
#include <sys/socket.h>
typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET (-1)
#define UNLINK unlink
#define closesocket close
static inline std::string getErrorMessage(void) { return std::to_string(errno) + " (" + std::string(strerror(errno)) + ")"; }
#endif

#include "common.h"
#include "tcp/wsa_init.hpp"


namespace sick_scansegment_xd
{
    /*
     * Shortcut to convert 4 byte to uint32t assuming little endian.
     */
    static inline uint32_t Convert4Byte(const uint8_t* p_data)
    {
        return (p_data[0]) | (p_data[1] << 8) | (p_data[2] << 16) | (p_data[3] << 24);
    }

    /*
    * Computes the zlib CRC32 checksum, see https://stackoverflow.com/questions/15030011/same-crc32-for-python-and-c
    */
    static inline uint32_t crc32(uint32_t crc, const uint8_t* buf, size_t len)
    {
        crc = ~crc;
        while (len--)
        {
            crc ^= *buf++;
            for (int k = 0; k < 8; k++)
                crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
        }
        return ~crc;
    }

    /*
     * @brief the udp socket to receive udp data
     */
    class UdpReceiverSocketImpl
    {
    public:
        /** Default constructor */
        UdpReceiverSocketImpl();

        /** Destructor, closes the socket */
        ~UdpReceiverSocketImpl();

        /** Opens a udp socket */
        bool Init(const std::string& udp_sender, int udp_port);

        /** Force stop a any blocking Receive()'s by enabling the exit condition and shutting down the recv socket */
        void ForceStop();

        /** Reads blocking until some data has been received successfully or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload);

        /** Reads blocking until all bytes of a msgpack incl. header and crc have been received or an error occurs. Returns the number of bytes received. */
        size_t Receive(std::vector<uint8_t>& msg_payload, double timeout, const std::vector<uint8_t>& udp_msg_start_seq);

        /** Return the udp port */
        inline int port(void) const { return m_udp_port; }

    protected:
        std::string m_udp_sender; // IP of udp sender
        int m_udp_port;           // udp port
        SOCKET m_udp_socket;      // udp raw socket
        bool m_force_quit;        // allows Receive() to unblock in case external code needs to stop running threads

    };

    /*!
     * @brief class UdpSenderSocketImpl implements the udp socket for sending udp packages
     */
    class UdpSenderSocketImpl
    {
    public:
        /*!
         * Constructor, opens an udp socket.
         * @param[in] server_address ip address
         * @param[in] udp_port udp port
         */
        UdpSenderSocketImpl(const std::string& server_address = "192.168.0.1", int udp_port = 2115);

        /*!
         * Destructor, closes the socket
         */
        ~UdpSenderSocketImpl();

        /*!
         * Returns true if the udp socket is opened and ready to send, or false otherwise.
         */
        inline bool IsOpen(void) const { return (m_udp_socket != INVALID_SOCKET); }

        /*!
         * Sends a binary message.
         */
        bool Send(std::vector<uint8_t>& message);

    protected:
        bool m_socket_opened;         // true if the udp socket is opened and ready to send, or false otherwise
        std::string m_server_address; // ip address
        int m_udp_port;               // udp port
        SOCKET m_udp_socket;          // udp raw socket

    };

};   // namespace sick_scansegment_xd
