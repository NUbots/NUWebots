// File:          NUgus_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

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

    // Setup arguments for select call
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(tcp_fd, &rfds);
    timeval timeout = {0, 0};

    // Current message counter
    uint32_t current_num = 0;

    while (current_num < 10) {
        // Watch TCP file descriptor to see when it has input.
        // No wait - polling as fast as possible
        int num_ready = select(tcp_fd + 1, &rfds, nullptr, nullptr, &timeout);
        if (num_ready < 0) {
            std::cerr << "Error: Polling of TCP connection failed: " << strerror(errno) << std::endl;
            continue;
        }
        else if (num_ready > 0) {
            // Wire format
            // unit64_t N   message size in bytes
            // uint8_t * N  the message
            uint64_t N;
            if (recv(tcp_fd, &N, sizeof(N), 0) != sizeof(N)) {
                std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno) << std::endl;
                continue;
            }

            std::vector<uint8_t> data(N, 0);
            if (recv(tcp_fd, data.data(), N, 0) != N) {
                std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno) << std::endl;
                continue;
            }

            controller::nugus::RobotControl msg;
            if (msg.ParseFromArray(data.data(), N)) {
                std::cerr << "Error: Failed to parse serialised message" << std::endl;
                continue;
            }

            std::cout << "Received message: " << msg.num() << " -- " << current_num << std::endl;

            // Read out the current message counter from the message
            current_num = msg.num();

            // Increment the counter and send the message back
            current_num += 1;
            msg.set_num(current_num);
            std::string output_data = msg.SerializeAsString();
            N                       = output_data.size();
            send(tcp_fd, &N, sizeof(N), 0);
            send(tcp_fd, data.data(), output_data.size(), 0);
        }
    }

#ifdef _WIN32
    closesocket(tcp_fd);
#else
    close(tcp_fd);
#endif

    return 0;
}
