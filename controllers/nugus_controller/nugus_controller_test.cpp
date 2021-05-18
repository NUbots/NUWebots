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

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

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

#include "RobotControl.pb.h"

int connect(const std::string& server_address, const int& port) {
#ifdef _WIN32
    // initialize the socket api
    WSADATA info;

    // Winsock 1.1
    if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
        printf("cannot initialize Winsock\n");

        return -1;
    }
#endif

    // create the socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        std::cerr << "Cannot create socket" << std::endl;
        return -1;
    }

    // fill in the socket address
    sockaddr_in address;
    std::memset(&address, 0, sizeof(sockaddr_in));
    address.sin_family = AF_INET;
    address.sin_port   = htons(port);
    hostent* server    = gethostbyname(server_address.data());

    if (server)
        std::memcpy(reinterpret_cast<char*>(&address.sin_addr.s_addr),
                    reinterpret_cast<char*>(server->h_addr),
                    server->h_length);
    else {
        std::cerr << "Cannot resolve server name: " << server_address << std::endl;
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
        return -1;
    }

    // connect to the server
    if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr)) == -1) {
        std::cerr << "Cannot connect to the server" << std::endl;
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
        return -1;
    }
    else {
        std::cerr << "Successfully connected to server" << std::endl;
    }

    return fd;
}

int main(int argc, char** argv) {
    // Make sure we have the command line arguments we need
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <TCP ADDRESS> <TCP PORT>" << std::endl;
        return 0;
    }

    // Connect to the server
    int tcp_fd = -1;
    while (tcp_fd == -1) {
        tcp_fd = connect(argv[1], std::stoi(argv[2]));
    }

    // Do nothing with the welcome message sent
    char welcome_message[8];
    recv(tcp_fd, welcome_message, sizeof(welcome_message), 0);

    // Current message counter
    uint32_t current_num = 1;

    // Send initial message
    controller::nugus::RobotControl msg;
    msg.set_num(current_num);

    uint32_t Nh = msg.ByteSizeLong();
    std::vector<uint8_t> data(Nh, 0);
    msg.SerializeToArray(data.data(), Nh);

    uint32_t Nn = htonl(Nh);

    if (send(tcp_fd, &Nn, sizeof(Nn), 0) < 0) {
        std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
    }
    if (send(tcp_fd, data.data(), data.size(), 0) < 0) {
        std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
    }

    while (current_num <= 10) {
        // Setup arguments for select call
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(tcp_fd, &rfds);
        timeval timeout = {0, 0};

        // Watch TCP file descriptor to see when it has input.
        // No wait - polling as fast as possible
        int num_ready = select(tcp_fd + 1, &rfds, nullptr, nullptr, &timeout);
        if (num_ready < 0) {
            std::cerr << "Error: Polling of TCP connection failed: " << strerror(errno) << std::endl;
        }
        else if (num_ready > 0) {
            // Wire format
            // unit32_t Nn  message size in bytes. The bytes are in network byte order (big endian)
            // uint8_t * Nn  the message
            if (recv(tcp_fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno) << std::endl;
                continue;
            }

            uint32_t Nh = ntohl(Nn);

            if (recv(tcp_fd, data.data(), Nh, 0) != Nh) {
                std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno) << std::endl;
                continue;
            }

            if (!msg.ParseFromArray(data.data(), Nh)) {
                std::cerr << "Error: Failed to parse serialised message" << std::endl;
                continue;
            }

            std::cout << "Received message: " << msg.num() << " -- " << current_num << std::endl;

            // Read out the current message counter from the message
            current_num = msg.num();

            // Increment the counter and send the message back
            current_num += 1;
            msg.set_num(current_num);

            Nh = msg.ByteSizeLong();
            data.resize(Nh);
            msg.SerializeToArray(data.data(), Nh);

            // Covert to network endianness, which might be different to host endianness
            Nn = htonl(Nh);

            if (send(tcp_fd, &Nn, sizeof(Nn), 0) < 0) {
                std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
            }
            if (send(tcp_fd, data.data(), data.size(), 0) < 0) {
                std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
            }
        }
    }

#ifdef _WIN32
    closesocket(tcp_fd);
#else
    close(tcp_fd);
#endif

    return 0;
}
