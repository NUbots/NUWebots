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
#include <memory>
#include <webots/Robot.hpp>

#include "utility/tcp.hpp"

using namespace utility::tcp;

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
    int server_port = 0;
    try {
        server_port = std::stoi(argv[1]);
    }
    catch (...) {
        std::cerr << "Failed to convert server port number to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Load in the simulation timestep from the command line and convert to an int
    int time_step = 0;
    try {
        time_step = std::stoi(argv[2]);
    }
    catch (...) {
        std::cerr << "Failed to convert simulation time step to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Start the TCP server
    int tcp_fd = create_socket_server(server_port);

    // Create the Robot instance
    std::unique_ptr<webots::Robot> robot = std::make_unique<webots::Robot>();

    // Run the robot controller
    while (robot->step(time_step) != -1) {
        // Don't bother doing anything unless we have an active TCP connection
        if (tcp_fd == -1) {
            std::cerr << "Error: Failed to start TCP server, retrying ..." << std::endl;
            tcp_fd = create_socket_server(server_port);
            continue;
        }

        // TODO: Do things ....
    }

    // Stop the TCP server
    close_socket(tcp_fd);

    return EXIT_SUCCESS;
}
