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
#include <array>
#include <filesystem>
#include "yaml-cpp/yaml.h"

#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>

const int QUALITY = 100;

int main(int argc, char** argv) {
    // ---------ARGUMENTS------------//
    // Make sure we have the command line arguments we need
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <DEF>" << std::endl;
        return EXIT_FAILURE;
    }
    // Load def argument which will be used to identify the robot using the webots getFromDef function
    std::string def = argv[1];
    
    //---------READ CONFIG-----------//
    // AxisAngle rotations will be read from a config file and saved here
    std::vector<std::array<double, 4>> rotations;

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

    // Counter for saving images
    int count = 0;
    
    //---------GET ROBOT, CAMERAS AND SUPERVISOR----------//
    // create the Supervisor instance and assign it to a robot
    webots::Supervisor supervisor = webots::Supervisor();
    webots::Node& robot          = *supervisor.getFromDef(def);

    // Get the time step of the current world.
    int time_step = int(supervisor.getBasicTimeStep());

    // Create the Robot instance
    // std::unique_ptr<webots::Robot> robot = std::make_unique<webots::Robot>();

    // Get the cameras
    std::unique_ptr<webots::Camera> right_camera = std::make_unique<webots::Camera>("right_camera");
    std::unique_ptr<webots::Camera> left_camera = std::make_unique<webots::Camera>("left_camera");
    right_camera->enable(time_step);
    left_camera->enable(time_step);
    right_camera->recognitionEnable(time_step);
    left_camera->recognitionEnable(time_step);
    right_camera->enableRecognitionSegmentation();
    left_camera->enableRecognitionSegmentation();

    // Create directories for saving data
    std::filesystem::create_directories("./data/data_normal");
    std::filesystem::create_directories("./data/data_stereo");

    //-----------SET RANDOM SEED-----------//
    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> xDistrib(0, 1080);
    std::uniform_int_distribution<> yDistrib(0, 760);

    //--------MAIN CONTROL LOOP------------//
    while (supervisor.step(time_step) != -1) {
        //----------SET TRANSLATION OF ROBOT------------//
        // Grab the current translation field of the robot to modify
        webots::Field& robot_translation_field = *robot.getField("translation");
        // Convert the field to a vector to output to console
        const double* robot_translation_vec = robot_translation_field.getSFVec3f();

        // 0, 0, 0 is centre of the playing field.
        // X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
        // from red
        // Y ranges from -3.8 - 3.8, the positive value on the red goal side
        // Z should be set to 0.51 to be on the ground
        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state
        std::array<double, 3> newPos;
        newPos[0] = 5.4 - xDistrib(gen) / 100;
        newPos[1] = 3.8 - yDistrib(gen) / 100;
        newPos[2] = 0.51;

        // Set new location
        robot_translation_field.setSFVec3f(newPos.data());

        // Grab the current rotation field of the robot to modify
        webots::Field& robot_rotation_field = *(robot.getField("rotation"));
        // Convert the field to a vector to output to console
        const double* robot_rotation_vec = robot_rotation_field.getSFRotation();
        
        // ---------LOOP OVER ROTATIONS--------//
        for (const std::array<double, 4>& rotation : rotations) {
            //-----------SAVE DATA-----------//
            // Save stereo images
            left_camera->saveImage("./data/data_stereo/image" + std::to_string(count) + "_L.jpeg", QUALITY);
            right_camera->saveImage("./data/data_stereo/image" + std::to_string(count) + "_R.jpeg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + std::to_string(count) + "_L.png", QUALITY);
            right_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + std::to_string(count) + "_R.png", QUALITY);
            
            // Save normal images            
            left_camera->saveImage("./data/data_normal/image0" + std::to_string(count) + ".jpeg", QUALITY);
            right_camera->saveImage("./data/data_normal/image0" + std::to_string(count) + ".jpeg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_normal/mask0" + std::to_string(count) + ".png", QUALITY);
            right_camera->saveRecognitionSegmentationImage("./data/data_normal/mask0" + std::to_string(count) + ".png", QUALITY);
            
            
            // Write the lens.yaml data
            std::ofstream lensFile("./data/data_stereo/lens" + std::to_string(count) + ".yaml");
            YAML::Emitter lensYaml;  // create the node
            lensYaml << YAML::BeginMap;
            lensYaml << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            lensYaml << YAML::Key << "focal_length" << YAML::Value << left_camera->getFocalLength();
            lensYaml << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "fov" << YAML::Value << left_camera->getFov();
            lensYaml << YAML::EndMap;
            
            lensFile << lensYaml.c_str();
            lensFile.close();

            count++;

            //-----------SET ROTATION----------//
            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly
            // Apply new rotation and reset physics to avoid robot tearing itself apart
            robot_rotation_field.setSFRotation(rotation.data());
            robot.resetPhysics();
            supervisor.step(time_step);
        }
    }
    return 0;
}
