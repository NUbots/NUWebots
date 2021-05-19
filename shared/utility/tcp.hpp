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
    #include <arpa/inet.h>   // definition of inet_ntoa
    #include <netdb.h>       // definition of gethostbyname
    #include <netinet/in.h>  // definition of struct sockaddr_in
    #include <poll.h>        // definition of poll and pollfd
    #include <sys/socket.h>  // definition of socket, accept, listen, and bind
    #include <unistd.h>      // definition of close
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

inline int create_socket_server(const int& port) {
#ifdef _WIN32
    // initialize the socket api
    WSADATA info;

    // Winsock 1.1
    int err = WSAStartup(MAKEWORD(1, 1), &info);
    switch (err) {
        case 0: break;
        case WSASYSNOTREADY:
            std::cerr << "Error: Cannot initialize Winsock: Network subsystem is not ready for communication (" << err
                      << ")" << std::endl;
        case WSAVERNOTSUPPORTED:
            std::cerr << "Error: Cannot initialize Winsock: Winsock version 1.1 is not supported (" << err << ")"
                      << std::endl;
        case WSAEINPROGRESS:
            std::cerr << "Error: Cannot initialize Winsock: A blocking operation is currently in progress (" << err
                      << ")" << std::endl;
        case WSAEPROCLIM:
            std::cerr << "Error: Cannot initialize Winsock: Process limit exceeded (" << err << ")" << std::endl;
        case WSAEFAULT:
            std::cerr << "Error: Cannot initialize Winsock: Invalid data pointer (" << err << ")" << std::endl;
        default: std::cerr << "Error: Cannot initialize Winsock: Unknown error (" << err << ")" << std::endl; return -1;
    }

#endif
    // create the socket
    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "Error: Cannot create socket: " << strerror(errno) << std::endl;
        return -1;
    }

    // fill in socket address
    sockaddr_in address;
    std::memset(&address, 0, sizeof(sockaddr_in));
    address.sin_family      = AF_INET;
    address.sin_port        = htons((unsigned short) port);
    address.sin_addr.s_addr = INADDR_ANY;

    // bind to port
    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr)) == -1) {
        std::cerr << "Error: Cannot bind port " << port << ": " << strerror(errno) << std::endl;
        close_socket(server_fd);
        return -1;
    }

    // listen for connections
    if (listen(server_fd, 1) == -1) {
        std::cerr << "Error: Cannot listen for connections: " << strerror(errno) << std::endl;
        close_socket(server_fd);
        return -1;
    }

    // Server is now set up and listening for connections
    std::cout << "Waiting for a connection on port " << port << " ..." << std::endl;

    return server_fd;
}

inline int check_for_connection(const int& server_fd, const int& port) {

    // Setup the polling data
    pollfd fds;
    fds.fd      = server_fd;
    fds.events  = POLLIN | POLLPRI;  // Check for data to read and urgent data to read
    fds.revents = 0;

    // Poll the server fd to see if there is any data to read
    const int num_ready = poll(&fds, 1, 0);

    // Polling failed
    if (num_ready < 0) {
        std::cerr << "Error: Polling of TCP connection failed: " << strerror(errno) << std::endl;
        return -1;
    }

    // We have an incoming connection
    else if (num_ready > 0) {
#ifdef _WIN32
        int asize = sizeof(sockaddr_in);
#else
        socklen_t asize = sizeof(sockaddr_in);
#endif

        // Accept the connection
        sockaddr_in client;
        const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client), &asize);

        // Failed to accept the connection
        if (client_fd == -1) {
            std::cerr << "Error: Cannot accept client connection on port " << port << ": " << strerror(errno)
                      << std::endl;
            close_socket(server_fd);
            return -1;
        }

        // Get client information
        const hostent* client_info = gethostbyname(inet_ntoa(client.sin_addr));
        std::cout << "Accepted connection on port " << port << " from: " << client_info->h_name << std::endl;

        // Return the client fd
        return client_fd;
    }

    std::cout << "Waiting for a connection on port " << port << " ..." << std::endl;

    // Nothing yet
    return 0;
}

}  // namespace utility::tcp

#endif  // NUBOTS_WEBOTS_UTILITY_TCP_HPP
