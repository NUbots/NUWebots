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
#include <Eigen/Core>
#include <Eigen/Geometry>

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
    std::uniform_int_distribution<> zDistrib(0, 5);

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
        // Z should be set to 0.51 to be on the ground (and add a bit of noise)
        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state
        std::array<double, 3> newPos;
        newPos[0] = 5.4 - xDistrib(gen) / 100;
        newPos[1] = 3.8 - yDistrib(gen) / 100;
        newPos[2] = 0.51; //- zDistrib(gen) / 100;  //TODO(wongjoel) the bit adding noise is commented out to make debugging easier

        // Set new location
        robot_translation_field.setSFVec3f(newPos.data());

        // Grab the current rotation field of the robot to modify
        webots::Field& robot_rotation_field = *(robot.getField("rotation"));
        // Convert the field to a vector to output to console
        const double* robot_rotation_vec = robot_rotation_field.getSFRotation();

        // ---------LOOP OVER ROTATIONS--------//
        for (const std::array<double, 4>& rotation : rotations) {

            //-----------SET ROTATION----------//
            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly
            // Apply new rotation and reset physics to avoid robot tearing itself apart
            robot_rotation_field.setSFRotation(rotation.data());
            robot.resetPhysics();

            // TODO(KipHamiltons) verify this is the right rotation - as in, verify is Roc, not Rco, and that the encoding of xyzw is the same order here
            // Get WORLD TO TORSO
            // World to torso transformation
            Eigen::Affine3d Htw;
            // TODO(wongjoel) verify these are the correct indexes
            Htw.linear() = Eigen::AngleAxisd(rotation[0], Eigen::Vector3d(rotation[1], rotation[2], rotation[3])).toRotationMatrix();
            Htw.translation() = Eigen::Vector3d(newPos.data());

            // Get TORSO TO NECK
            // relative to torso, torso to neck
            webots::Field* neck_translation = robot.getFromProtoDef("neck_solid")->getField("translation");

            // Rtn, torso to neck
            webots::Field* neck_rotation = robot.getFromProtoDef("neck_solid")->getField("rotation");

            // Homogenous transformation of the neck to the robot's torso
            Eigen::Affine3d Htn;
            Htn.translation() = Eigen::Vector3d(*neck_translation->getMFVec3f(0), *neck_translation->getMFVec3f(1), *neck_translation->getMFVec3f(2));

            // TODO(wongjoel) verify these are the correct indexes
            Htn.linear() = Eigen::AngleAxisd(
                *neck_rotation->getMFRotation(0),
                Eigen::Vector3d(*neck_rotation->getMFRotation(1), *neck_rotation->getMFRotation(2), *neck_rotation->getMFRotation(3))
            ).toRotationMatrix();

            // Get NECK TO CAMERA
            webots::Field* camera_translation = robot.getFromProtoDef("right_camera")->getField("translation");
            webots::Field* camera_rotation = robot.getFromProtoDef("right_camera")->getField("rotation");
            
            Eigen::Affine3d Hnc;
            Hnc.translation() = Eigen::Vector3d(*camera_translation->getMFVec3f(0), *camera_translation->getMFVec3f(1), *camera_translation->getMFVec3f(2));

            // TODO(wongjoel) verify these are the correct indexes
            Hnc.linear() = Eigen::AngleAxisd(
                *camera_rotation->getMFRotation(0),
                Eigen::Vector3d(*camera_rotation->getMFRotation(1), *camera_rotation->getMFRotation(2), *camera_rotation->getMFRotation(3))
            ).toRotationMatrix();

            Eigen::Affine3d Hwc = Htw.inverse() * Htn * Hnc;

            // Hoc.linear() = Roc.matrix();
            // // TODO(YsobelSims) should this vector be negated?? this might be rWCc or rCWc or some other thing kip didn't think of
            // Hoc.translation() = Eigen::Vector3d(newPos.data());

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
            lensYaml << YAML::Key << "Hoc" << YAML::Value;
            lensYaml << YAML::BeginSeq;
            //TODO(wongjoel) this emits a list instead of an array, need to spit out an array in format [0.0 0.0 0.0 0.0]
            lensYaml << YAML::Flow << std::vector({Hwc(0, 0), Hwc(0, 1), Hwc(0, 2), newPos[0]});
            lensYaml << YAML::Flow << std::vector({Hwc(1, 0), Hwc(1, 1), Hwc(1, 2), newPos[1]});
            lensYaml << YAML::Flow << std::vector({Hwc(2, 0), Hwc(2, 1), Hwc(2, 2), newPos[2]});
            lensYaml << YAML::Flow << YAML::BeginSeq << 0 << 0 << 0 << 1 << YAML::EndSeq;
            lensYaml << YAML::EndSeq;
            lensYaml << YAML::EndMap;

            lensFile << lensYaml.c_str();
            lensFile.close();

            count++;
            supervisor.step(time_step);
        }
    }
    return 0;
}
