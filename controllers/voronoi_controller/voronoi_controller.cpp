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
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <yaml-cpp/yaml.h>

#include "utility/tcp.hpp"

using namespace utility::tcp;

    // Randomly find robot positions that do not clash and return those positions
    std::vector<std::array<double, 3>> find_robot_positions(const std::vector<webots::Node*>& robot_nodes,
                                                            //std::uniform_real_distribution<> x_distrib,
                                                            //std::uniform_real_distribution<> y_distrib,
                                                            //std::mt19937 gen,
                                                            const double min_distance) {
        // Loop through every robot and find valid teleport locations
        std::vector<std::array<double, 3>> positions = {};

        for (const auto& robot : robot_nodes) {
            // Flag for if robots have collided. Start by assuming no collision, and then update if there is any collision.
            bool collision = false;
            // new_pos will be a "Proposed location" for a robot to teleport to
            std::array<double, 3> new_pos{};
            do {
                collision = false;
                // Generate a new random location
                new_pos[0] = x_distrib(gen);
                new_pos[1] = y_distrib(gen);
                new_pos[2] = robot->getField("height")->getSFFloat();

                // Loop through the vector of existing proposed locations and see if the new one is going to
                // collide with any of them
                for (const auto& testPos : positions) {
                    const double distance =
                        std::sqrt(std::pow((new_pos[1] - testPos[1]), 2) + std::pow((new_pos[0] - testPos[0]), 2));
                    collision = distance < min_distance ? true : collision;
                }
                // Loop until a proposed location has been found that doesn't clash with the existing ones
            } while (collision);
            // Finally add the proposed location in as a confirmed position
            positions.emplace_back(new_pos);
        }
        return positions;

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
        return EXIT_FAILURE;
    }

    // GET SUPERVISOR, ROBOTS AND TIME STEP
    // Load def arguments of other robots in the field into a vector
    std::vector<webots::Node*> robot_nodes;
    // Only one supervisor instance can exist in a single controller. As such the same one will be used
    // every time
    webots::Supervisor supervisor = webots::Supervisor();
    for (int i = 2; i < argc; i++) {
        // Load the Node of the robot by def argument and add to vector
        robot_nodes.emplace_back(supervisor.getFromDef(argv[i]));
    }
    // Get the time step of the current world.
    // int time_step = int(supervisor.getBasicTimeStep());

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
