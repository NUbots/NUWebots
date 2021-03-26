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
                return 0;
            }

            // Load in the TCP port number from the command line and convert to an int
            int server_port;
            try {
                server_port = std::stoi(argv[1]);
            }
            catch (...) {
                std::cerr << "Failed to convert server port number to an integer" << std::endl;
                return 0;
            }

            // Load in the simulation timestep from the command line and convert to an int
            int time_step;
            try {
                time_step = std::stoi(argv[2]);
            }
            catch (...) {
                std::cerr << "Failed to convert simulation time step to an integer" << std::endl;
                return 0;
            }

            // Start the TCP server
            int tcp_fd = create_socket_server(server_port);
            if (tcp_fd == -1) {
                std::cerr << "Error: Failed to start TCP server" << std::endl;
                return 0;
            }

            // Create the Robot instance
            std::unique_ptr<webots::Robot> robot = std::make_unique<webots::Robot>();

            // Run the robot controller
            while (robot->step(time_step) != -1) {
                // Don't bother doing anything unless we have an active TCP connection
                if (tcp_fd == -1) {
                    tcp_fd = create_socket_server(server_port);
                    continue;
                }

                // TODO: Do things ....
            }

            // Stop the TCP server
            close_socket(tcp_fd);

            return 0;
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
