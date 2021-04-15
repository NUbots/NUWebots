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

#include <fstream>
#include <iostream>
#include <random>
#include <webots/Supervisor.hpp>
#include <array>

#include "yaml-cpp/yaml.h"

// AxisAngle rotations will be read from a config file and saved here
std::vector<std::array<double, 4>> rotations;

int main(int argc, char** argv) {

    // Make sure we have the command line arguments we need
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <DEF>" << std::endl;
        return EXIT_FAILURE;
    }
    // Load def argument which will be used to identify the robot using the webots getFromDef function
    std::string def = argv[1];
    

    // Load config file
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        rotations         = config["rotations"].as<std::vector<std::array<double, 4>>>();
    }
    catch (const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return 1;
    }
    catch (const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return 2;
    }
    
    // create the Supervisor instance and assign it to a robot
    webots::Supervisor supervisor = webots::Supervisor();
    webots::Node& target          = *supervisor.getFromDef(def);

    // Get the time step of the current world.
    int time_step = int(supervisor.getBasicTimeStep());

    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> xDistrib(0, 1080);
    std::uniform_int_distribution<> yDistrib(0, 760);

    while (supervisor.step(time_step) != -1) {

        // Grab the current translation field of the robot to modify
        webots::Field& target_translation_field = *target.getField("translation");
        // Convert the field to a vector to output to console
        const double* target_translation_vec = target_translation_field.getSFVec3f();
        
        // Prepare new location

        // 0, 0, 0 is centre of the playing field.
        // X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
        // from red
        // Y ranges from -8 - 8, the positive value on the red goal side
        // Z should be set to 0.32 to be on the ground
        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state


        std::array<double, 3> newPos;
        newPos[0] = 5.4 - xDistrib(gen) / 100;
        newPos[1] = 3.8 - yDistrib(gen) / 100;
        newPos[2] = 0.51;

        // Set new location
        target_translation_field.setSFVec3f(newPos.data());

        // Grab the current rotation field of the robot to modify
        webots::Field& target_rotation_field = *(target.getField("rotation"));
        // Convert the field to a vector to output to console
        const double* target_rotation_vec = target_rotation_field.getSFRotation();
        
        // Loop once for each rotation
        for (const std::array<double, 4>& rotation : rotations) {      
            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly

            // Apply new rotation and reset physics to avoid robot tearing itself apart
            target_rotation_field.setSFRotation(rotation.data());
            target.resetPhysics();
            supervisor.step(time_step);
        }
    };
    return 0;
}
