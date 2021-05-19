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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <poll.h>  // definition of poll and pollfd
#include <string>
#include <vector>
#include <webots/Robot.hpp>

#include "RobotControl.pb.h"

#include "utility/tcp.hpp"

using namespace utility::tcp;

class NUgus : public webots::Robot {
public:
    NUgus(const int& time_step, const int& server_port)
        : time_step(time_step), server_port(server_port), server_fd(create_socket_server(server_port)), client_fd(-1) {}
    ~NUgus() {
        close_socket(client_fd);
        close_socket(server_fd);
    }

    void run() {
        // Message counter
        uint32_t current_num = 1;

        while (step(time_step) != -1) {
            // Make sure we have a server
            if (server_fd == -1) {
                std::cerr << "Error: Lost TCP server, retrying ..." << std::endl;
                server_fd = create_socket_server(server_port);

                // If we had to recreate the server then the client is no longer valid
                close_socket(client_fd);
                client_fd = -1;
            }

            // Make sure we have an active TCP connection
            if (server_fd != -1 && client_fd == -1) {
                std::cerr << "Warning: No active TCP connection, retrying ..." << std::endl;
                client_fd = check_for_connection(server_fd, server_port);

                // There was an error accepting the new connection, our server is no longer valid
                if (client_fd < 0) {
                    server_fd = -1;
                }
                // No new connection came in, we are still waiting
                else if (client_fd == 0) {
                    client_fd = -1;
                }
                // We just accepted a new connection, send our welcome message
                else {
                    send(client_fd, "Welcome", 8, 0);
                }
            }

            // Server is good and client is good
            if (server_fd != -1 && client_fd != -1) {
                // Setup arguments for poll call
                pollfd fds;
                fds.fd      = client_fd;
                fds.events  = POLLIN | POLLPRI;  // Check for data to read and urgent data to read
                fds.revents = 0;

                // Watch TCP file descriptor to see when it has input.
                // No wait - polling as fast as possible
                int num_ready = poll(&fds, 1, 0);
                if (num_ready < 0) {
                    std::cerr << "Error: Polling of TCP connection failed: " << strerror(errno) << std::endl;
                    continue;
                }
                else if (num_ready > 0) {
                    // Wire format
                    // unit32_t Nn  message size in bytes. The bytes are in network byte order (big endian)
                    // uint8_t * Nn  the message
                    uint32_t Nn;
                    if (recv(client_fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                        std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno)
                                  << std::endl;
                        continue;
                    }

                    // Covert to host endianness, which might be different to network endianness
                    uint32_t Nh = ntohl(Nn);

                    std::vector<uint8_t> data(Nh, 0);
                    if (recv(client_fd, data.data(), Nh, 0) != Nh) {
                        std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno)
                                  << std::endl;
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

                    if (send(client_fd, &Nn, sizeof(Nn), 0) < 0) {
                        std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
                    }
                    else if (send(client_fd, data.data(), data.size(), 0) < 0) {
                        std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
                    }
                }
            }
        }
    }

private:
    /// Controller time step
    const int time_step;
    /// TCP server port
    const int server_port;
    /// File descriptor to use for the TCP server
    int server_fd;
    /// File descriptor to use for the TCP connection
    int client_fd;
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
