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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */
 
// Special thanks to Hamburg Bit-Bots for ideas for this controller
// https://github.com/bit-bots/wolfgang_robot/blob/feature/recognition/wolfgang_webots_sim/src/wolfgang_webots_sim/webots_camera_controller.py

#include <array>
#include <fstream>
#include <iostream>
#include <cmath>
#include <random>
#include <webots/Supervisor.hpp>

#include <yaml-cpp/yaml.h>

// AxisAngle rotations will be read from a config file and saved here
std::vector<std::array<double, 4>> rotations;
double minDistance;
double zHeight;
double xSize;
double ySize;

int main(int argc, char** argv) {

    // Make sure we have the command line arguments we need. At a minimum we should have the def argument
    // of the robot supervisor. Other arguments are def arguments of robots that should be controlled by
    // this supervisor
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <DEF>" << std::endl;
        return EXIT_FAILURE;
    }
    // Load self-def argument which will be used to identify the robot using the webots getFromDef function
    std::string def = argv[1];

    // Load def arguments of other robots in the field into a vector
    std::vector<webots::Node*> otherRobotsNodes;
    // Only one supervisor instance can exist in a single controller. As such the same one will be used
    // every time
    webots::Supervisor supervisor = webots::Supervisor();
    for (int i = 2; i < argc; i++) {
        // Load the Node of the robot by def argument and add to vector
        otherRobotsNodes.emplace_back(supervisor.getFromDef(argv[i]));
    }

    // Load config file
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        rotations         = config["rotations"].as<std::vector<std::array<double, 4>>>();
        minDistance       = config["minDistance"].as<double>();
        zHeight           = config["zHeight"].as<double>();
    }
    catch (const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    catch (const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 2;
    }

    webots::Node* tempNode = supervisor.getFromDef("test");
    const double xSize                  = tempNode->getField("xSize")->getSFFloat();
    const double ySize                  = tempNode->getField("ySize")->getSFFloat();

    // Get the time step of the current world.
    int timeStep = int(supervisor.getBasicTimeStep());

    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    // Generate distributions for random number systems in the following loop
    std::uniform_real_distribution<> xDistrib(-xSize * 0.5, xSize * 0.5);
    std::uniform_real_distribution<> yDistrib(-ySize * 0.5, ySize * 0.5);
    std::uniform_real_distribution<> rotDistrib(0.0, rotations.size() - 1.0);

    while (supervisor.step(timeStep) != -1) {

        // Declare a vector of positions that will be saved as they are randomly generated, to be later
        // applied to each robot
        std::vector<std::array<double, 3>> positions = std::vector<std::array<double, 3>>();
        // Loop through every robot
        for (size_t i = 0; i < otherRobotsNodes.size(); i++) {
            // Assume there is no collision
            bool collision = false;
            // newPos will be a "Proposed location" for a robot to teleport to
            std::array<double, 3> newPos;
            do {
                collision = false;
                // Generate a new random location
                newPos[0] = xDistrib(gen);
                newPos[1] = yDistrib(gen);
                newPos[2] = zHeight;

                // Loop through the vector of existing proposed locations and see if the new one is going to
                // collide with any of them
                for (std::array<double, 3> testPos : positions) {
                    double distance = std::sqrt(std::pow((newPos[1] - testPos[1]), 2) + std::pow((newPos[0] - testPos[0]), 2));
                    if (distance < minDistance) {
                        collision = true;
                    }
                }
            // Loop until a proposed location has been found that doesn't clash with the existing ones
            } while (collision);
            // Finally add the proposed location in as a confirmed position
            positions.push_back(newPos);
        }

        // Loop through every robot
        for (size_t i = 0; i < otherRobotsNodes.size(); i++) {
            // Grab translation field of the robot to modify
            webots::Field* target_translation_field = otherRobotsNodes[i]->getField("translation");
            // Grab the current rotation field of the robot to modify
            webots::Field* target_rotation_field = otherRobotsNodes[i]->getField("rotation");

            // There will be a position for every robot in the positions vector
            target_translation_field.setSFVec3f(positions[i].data());
            // Apply new rotation
            target_rotation_field.setSFRotation(rotations[rotDistrib(gen)].data());

            // Reset physics to avoid robot tearing itself apart
            otherRobotsNodes[i]->resetPhysics();
        }
    };
    return 0;
}
