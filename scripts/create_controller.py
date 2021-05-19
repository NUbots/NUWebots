#! /usr/bin/env python3

import os
import sys
import textwrap

if __name__ == "__main__":
    controller_name = sys.argv[1]
    controller_path = os.path.join("controllers", controller_name)

    # Make controller source directory
    try:
        os.makedirs(controller_path, exist_ok=False)
    except:
        print("Unable to create new controller directory as it already exists.")
        sys.exit(0)

    cmake_template = textwrap.dedent(
        """\
        # Find the webots package
        find_package(webots REQUIRED)

        # Find/list all of the source files
        set(controller_sources "{controller_name}.cpp")

        # Create an executable using all of the sources
        add_executable({controller_name} ${{controller_sources}})

        # Make sure we can find the utility folder
        target_include_directories({controller_name} PRIVATE "${{PROJECT_SOURCE_DIR}}/shared")

        # Link against the webots target (this will also add any necessary include directories to our target)
        target_link_libraries({controller_name} PRIVATE webots::webots)

        # Generate binary in controller source directory
        set_target_properties({controller_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${{CMAKE_CURRENT_SOURCE_DIR}}"
                                                OUTPUT_NAME {controller_name})
        """
    )

    controller_template = textwrap.dedent(
        """\
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
        #include <poll.h>  // definition of poll and pollfd
        #include <webots/Robot.hpp>

        #include "utility/tcp.hpp"

        using utility::tcp::check_for_connection;
        using utility::tcp::close_socket;
        using utility::tcp::create_socket_server;

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
            int server_fd = create_socket_server(server_port);
            int client_fd = -1;

            // Create the Robot instance
            std::unique_ptr<webots::Robot> robot = std::make_unique<webots::Robot>();

            // Run the robot controller
            while (robot->step(time_step) != -1) {
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
                        // TODO: Do things ....
                    }
                }
            }

            // Stop the TCP server
            close_socket(client_fd);
            close_socket(server_fd);

            return EXIT_SUCCESS;
        }
        """
    )

    # Write controller CMakeLists.txt
    with open(os.path.join(controller_path, "CMakeLists.txt"), "w") as f:
        f.write(cmake_template.format(controller_name=controller_name))

    # Write controller source file
    with open(os.path.join(controller_path, f"{controller_name}.cpp"), "w") as f:
        f.write(controller_template)

    # Add controller directory to the controllers CMakeList.txt
    with open(os.path.join(controller_path, "..", "CMakeLists.txt"), "a") as f:
        f.write(f"add_subdirectory({controller_name})\n")

    # Append new controller binary to .gitignore
    with open(".gitignore", "a") as f:
        f.write("{}\n".format(os.path.join(os.sep, controller_path, controller_name)))
