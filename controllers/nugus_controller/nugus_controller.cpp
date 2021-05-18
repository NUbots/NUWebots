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
// and/or add some other includes
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <webots/Robot.hpp>

#include "RobotControl.pb.h"

#include "utility/tcp.hpp"

using namespace utility::tcp;

class NUgus : public webots::Robot {
public:
    NUgus(const int& time_step, const int& server_port)
        : time_step(time_step), server_port(server_port), tcp_fd(create_socket_server(server_port)) {
        send(tcp_fd, "Welcome", 8, 0);
    }
    ~NUgus() {
        close_socket(tcp_fd);
    }

    void run() {
        // Message counter
        uint32_t current_num = 1;

        while (step(time_step) != -1) {
            // Don't bother doing anything unless we have an active TCP connection
            if (tcp_fd == -1) {
                std::cerr << "Error: Failed to start TCP server, retrying ..." << std::endl;
                tcp_fd = create_socket_server(server_port);
                send(tcp_fd, "Welcome", 8, 0);
                continue;
            }

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
                continue;
            }
            else if (num_ready > 0) {
                // Wire format
                // unit32_t Nn  message size in bytes. The bytes are in network byte order (big endian)
                // uint8_t * Nn  the message
                uint32_t Nn;
                if (recv(tcp_fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                    std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno)
                              << std::endl;
                    continue;
                }

                // Covert to host endianness, which might be different to network endianness
                uint32_t Nh = ntohl(Nn);

                std::vector<uint8_t> data(Nh, 0);
                if (recv(tcp_fd, data.data(), Nh, 0) != Nh) {
                    std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno) << std::endl;
                    continue;
                }

                // Parse message data
                controller::nugus::RobotControl msg;
                if (!msg.ParseFromArray(data.data(), Nh)) {
                    std::cerr << "Error: Failed to parse serialised message" << std::endl;
                    continue;
                }

                // Read out the current message counter from the message
                current_num = msg.num();

                // Send a message to the client
                msg.set_num(current_num);

                Nh = msg.ByteSizeLong();
                data.resize(Nh);
                msg.SerializeToArray(data.data(), Nh);

                Nn = htonl(Nh);

                if (send(tcp_fd, &Nn, sizeof(Nn), 0) < 0) {
                    std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
                }
                else if (send(tcp_fd, data.data(), data.size(), 0) < 0) {
                    std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
                }
            }
        }
    }

private:
    /// Controller time step
    const int time_step;
    /// TCP server port
    const int server_port;
    /// File descriptor to use for the TCP connection
    int tcp_fd;
};


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
        return EXIT_FAILURE;
    }

    // Load in the TCP port number from the command line and convert to an int
    int server_port;
    try {
        server_port = std::stoi(argv[1]);
    }
    catch (...) {
        std::cerr << "Failed to convert server port number to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Load in the simulation timestep from the command line and convert to an int
    int time_step;
    try {
        time_step = std::stoi(argv[2]);
    }
    catch (...) {
        std::cerr << "Failed to convert simulation time step to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Create the Robot instance and initialise the TCP connection
    std::unique_ptr<NUgus> nugus = std::make_unique<NUgus>(time_step, server_port);

    // Run the robot controller
    nugus->run();

    return EXIT_SUCCESS;
}
