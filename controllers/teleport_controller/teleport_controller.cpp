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

#include <RobotisOp2MotionManager.hpp>
#include <fstream>
#include <iostream>
#include <random>
#include <webots/Supervisor.hpp>

#include "yaml-cpp/yaml.h"

// AxisAngle rotations will be read from a config file and saved here
std::vector<std::vector<double>> rotations;
// Preset motions for the robot will be read and saved here
std::vector<int> playPages;

int main() {

    // Load config file
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        rotations         = config["rotations"].as<std::vector<std::vector<double>>>();
        playPages         = config["playpages"].as<std::vector<int>>();
    }
    catch (const std::string& e) {
        std::cout << e << std::endl;
        return 0;
    }

    // create the Supervisor instance and assign it to a robot
    webots::Supervisor supervisor = webots::Supervisor();
    webots::Node& target          = *supervisor.getFromDef("RED_1");

    // Get the time step of the current world.
    int timeStep = int(supervisor.getBasicTimeStep());

    // Add the motion manager to make the robot stand up and look around
    managers::RobotisOp2MotionManager mMotionManager = managers::RobotisOp2MotionManager(&supervisor);
    mMotionManager.playPage(1);  // Look around

    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> xDistrib(0, 1080);
    std::uniform_int_distribution<> yDistrib(0, 1600);

    while (supervisor.step(timeStep) != -1) {

        // Grab the current translation field of the robot to modify
        webots::Field& target_translation_field = *target.getField("translation");
        // Convert the field to a vector to output to console
        const double* target_translation_vec = target_translation_field.getSFVec3f();

        // Output current location
        try {
            std::ofstream log;
            log.open("log.txt", std::fstream::app);
            log << "Location: " << std::endl;
            log << "X: " << target_translation_vec[0];
            log << " Y: " << target_translation_vec[1];
            log << " Z: " << target_translation_vec[2] << std::endl;
            log.close();
        }
        catch (const std::string& e) {
            std::cout << e << std::endl;
        }
        // Prepare new location

        // 0, 0, 0 is centre of the playing field.
        // X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
        // from red
        // Y ranges from -8 - 8, the positive value on the red goal side
        // Z should be set to 0.32 to be on the ground
        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state


        double newPos[3];
        newPos[0] = 5.4 - xDistrib(gen) / 100;
        newPos[1] = 8 - yDistrib(gen) / 100;
        newPos[2] = 0.32;

        // Set new location
        target_translation_field.setSFVec3f(newPos);

        // Grab the current rotation field of the robot to modify
        webots::Field& target_rotation_field = *target.getField("rotation");
        // Convert the field to a vector to output to console
        const double* target_rotation_vec = target_rotation_field.getSFRotation();

        // Loop once for each rotation
        for (long unsigned int i = 0; i < rotations.size(); i++) {
            // Output current rotation
            try {
                std::ofstream log;
                log.open("log.txt", std::fstream::app);
                log << "Rotation: " << std::endl;
                log << "X: " << target_rotation_vec[0];
                log << " Y: " << target_rotation_vec[1];
                log << " Z: " << target_rotation_vec[2];
                log << " α: " << target_rotation_vec[3] << std::endl;
                log.close();
            }
            catch (const std::string& e) {
                std::cout << e << std::endl;
            }

            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly

            // The setSFRotation webots function expects an array of doubles, not a vector
            double newRot[4];
            for (long unsigned int j = 0; j < rotations[i].size(); j++) {
                newRot[j] = rotations[i][j];
            }

            // Apply new rotation and reset physics to avoid robot tearing itself apart
            target_rotation_field.setSFRotation(newRot);
            target.resetPhysics();

            // The following performs a number of preset movements/routines loaded from the
            // config file that the robot can carry out. Any of them can be removed or more
            // added but leave at least one otherwise there will be no delay between each
            // step of this loop
            for (int i : playPages) {
                mMotionManager.playPage(i);
            }
        }
    };
    return 0;
}
