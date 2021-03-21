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
#include <memory>
#include <string>
#include <vector>
#include <webots/Robot.hpp>

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

class NUgus : public webots::Robot {
public:
    NUgus(const int& time_step, const int& tcp_fd) : time_step(time_step), tcp_fd(tcp_fd) {}
    ~NUgus() {
#ifdef _WIN32
        closesocket(tcp_fd);
        WSACleanup();
#else
        close(tcp_fd);
#endif
    }

    void run() {
        // Setup arguments for select call
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(tcp_fd, &rfds);
        timeval timeout = {0, 0};

        // Message counter
        uint32_t current_num = 0;

        while (step(time_step) != -1) {
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
                    std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno)
                              << std::endl;
                    continue;
                }

                std::vector<uint8_t> data(N, 0);
                if (recv(tcp_fd, data.data(), N, 0) != N) {
                    std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno) << std::endl;
                    continue;
                }

                // Parse message data
                controller::nugus::RobotControl msg;
                if (msg.ParseFromArray(data.data(), N)) {
                    std::cerr << "Error: Failed to parse serialised message" << std::endl;
                    continue;
                }

                // Read out the current message counter from the message
                current_num = msg.num();
            }

            std::cerr << "Sending message to client" << std::endl;

            // Send a message to the client
            controller::nugus::RobotControl msg;
            msg.set_num(current_num);
            std::string data = msg.SerializeAsString();
            uint64_t N       = data.size();
            send(tcp_fd, &N, sizeof(N), 0);
            send(tcp_fd, data.data(), data.size(), 0);
        }
    }

private:
    /// Controller time step
    const int time_step;
    /// File descriptor to use for the TCP connection
    const int tcp_fd;
};

int create_socket_server(const int& port) {
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
    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd == -1) {
        std::cerr << "Cannot create socket" << std::endl;
        return -1;
    }

    // fill in socket address
    sockaddr_in address;
    std::memset(&address, 0, sizeof(sockaddr_in));
    address.sin_family      = AF_INET;
    address.sin_port        = htons((unsigned short) port);
    address.sin_addr.s_addr = INADDR_ANY;

    // bind to port
    if (bind(sfd, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr)) == -1) {
        std::cerr << "Cannot bind port " << port << std::endl;
#ifdef _WIN32
        closesocket(sfd);
#else
        close(sfd);
#endif
        return -1;
    }

    // listen for connections
    if (listen(sfd, 1) == -1) {
        std::cerr << "Cannot listen for connections" << std::endl;
#ifdef _WIN32
        closesocket(sfd);
#else
        close(sfd);
#endif
        return -1;
    }
    std::cerr << "Waiting for a connection on port " << port << " ..." << std::endl;

#ifdef _WIN32
    int asize = sizeof(sockaddr_in);
#else
    socklen_t asize = sizeof(sockaddr_in);
#endif

    sockaddr_in client;
    int cfd = accept(sfd, reinterpret_cast<sockaddr*>(&client), &asize);

    if (cfd == -1) {
        std::cerr << "Cannot accept client" << std::endl;
    }
    else {
        hostent* client_info = gethostbyname(inet_ntoa(client.sin_addr));
        std::cerr << "Accepted connection from: " << client_info->h_name << std::endl;
    }

    return cfd;
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
    // Make sure we have the command line arguments we need
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <TCP PORT> <CONTROLLER_TIME_STEP>" << std::endl;
        return 0;
    }

    // Load in the server port from the command line and conver to an int
    const int server_port = std::stoi(argv[1]);
    const int time_step   = std::stoi(argv[2]);

    // Create the Robot instance and initialise the TCP connection
    std::unique_ptr<NUgus> nugus = std::make_unique<NUgus>(time_step, create_socket_server(server_port));

    // Run the robot controller
    nugus->run();

    return 0;
}
