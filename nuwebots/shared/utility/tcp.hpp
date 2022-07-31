/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@newcastle.edu.au>
 */
#ifndef NUBOTS_WEBOTS_UTILITY_TCP_HPP
#define NUBOTS_WEBOTS_UTILITY_TCP_HPP

#include <cstring>
#include <iostream>

// Include headers needed for TCP connection
#ifdef _WIN32
    #include <winsock.h>
#else
    #include <arpa/inet.h>  /* definition of inet_ntoa */
    #include <netdb.h>      /* definition of gethostbyname */
    #include <netinet/in.h> /* definition of struct sockaddr_in */
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <unistd.h> /* definition of close */
#endif

namespace utility::tcp {

    inline void close_socket(const int& tcp_fd) {
        if (tcp_fd > -1) {
#ifdef _WIN32
            closesocket(tcp_fd);
            WSACleanup();
#else
            close(tcp_fd);
#endif
        }
    }

    inline int create_socket_server(const uint16_t& port) {
#ifdef _WIN32
        // initialize the socket api
        WSADATA info;

        // Winsock 1.1
        if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
            std::cerr << "Cannot initialize Winsock" << std::endl;
            return -1;
        }
#endif
        // create the socket
        const int sfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sfd == -1) {
            std::cerr << "Cannot create socket" << std::endl;
            return -1;
        }

        // fill in socket address
        sockaddr_in address{};
        std::memset(&address, 0, sizeof(sockaddr_in));
        address.sin_family      = AF_INET;
        address.sin_port        = htons(port);
        address.sin_addr.s_addr = INADDR_ANY;

        // bind to port
        if (bind(sfd, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr)) == -1) {
            std::cerr << "Cannot bind port " << port << std::endl;
            close_socket(sfd);
            return -1;
        }

        // listen for connections
        if (listen(sfd, 1) == -1) {
            std::cerr << "Cannot listen for connections" << std::endl;
            close_socket(sfd);
            return -1;
        }
        std::cerr << "Waiting for a connection on port " << port << " ..." << std::endl;

#ifdef _WIN32
        int asize = sizeof(sockaddr_in);
#else
        socklen_t asize = sizeof(sockaddr_in);
#endif

        sockaddr_in client{};
        const int cfd = accept(sfd, reinterpret_cast<sockaddr*>(&client), &asize);

        if (cfd == -1) {
            std::cerr << "Cannot accept client" << std::endl;
            close_socket(sfd);
        }
        else {
            const hostent* client_info = gethostbyname(inet_ntoa(client.sin_addr));
            std::cerr << "Accepted connection from: " << client_info->h_name << std::endl;
        }

        return cfd;
    }

}  // namespace utility::tcp

#endif  // NUBOTS_WEBOTS_UTILITY_TCP_HPP
