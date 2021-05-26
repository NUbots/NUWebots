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
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <yaml-cpp/yaml.h>


std::string padLeft(int number, int width) {
    std::stringstream ss;
    ss << std::setw(width) << std::setfill('0') << number;
    return ss.str();
}

// Quality of the images we are saving
const int QUALITY = 100;

void collect_vision_data(const webots::Node& robot) {
    // Counter for saving images
    int count = 0;

    // Get the cameras
    webots::Camera* left_camera  = robot.getCamera("left_camera");
    webots::Camera* right_camera = robot.getCamera("right_camera");

    // Calculate camera field of view
    int camera_width             = left_camera->getWidth();
    int camera_height            = left_camera->getHeight();
    const double camera_diagonal = std::sqrt(camera_width * camera_width + camera_height * camera_height);
    const double horizontal_fov  = left_camera->getFov();
    const double vertical_fov = 2 * std::atan(std::tan(horizontal_fov * 0.5) * (double(camera_height) / camera_width));
    const double focal_length_px = 0.5 * camera_height / std::tan(vertical_fov / 2.0);
    const double diagonal_fov    = 2 * std::atan(camera_diagonal / (2 * focal_length_px));

    //-----------SET RANDOM SEED-----------//
    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    // Move servos in the range [-PI/2, PI/2]
    std::uniform_real_distribution<> servoDistrib(-M_PI_2, M_PI_2);

    //-----------GET HANDLES TO NODES AND SERVOS-----------//
    // Get a handle to our head and neck motors
    webots::Motor* neck_yaw                   = robot.getMotor("neck_yaw");
    webots::Motor* head_pitch                 = robot.getMotor("head_pitch");
    webots::PositionSensor* neck_yaw_sensor   = robot.getPositionSensor("neck_yaw_sensor");
    webots::PositionSensor* head_pitch_sensor = robot.getPositionSensor("head_pitch_sensor");

    //----------GET TRANSLATION OF ROBOT------------//
    webots::Field* robotTranslationField = robot.getField("translation");
    const double* robot_position = robotTranslationField->getSFVec3f();

    // ---------GET ROTATION OF ROBOT--------//
    webots::Field* robotRotationField = robot.getField("rotation");
    const double* rotation = robotRotationField->getSFRotation();

    // Set random positions for the head and neck servos
    const double neck_yaw_position   = servoDistrib(gen);
    const double head_pitch_position = servoDistrib(gen);
    neck_yaw->setPosition(neck_yaw_position);
    head_pitch->setPosition(head_pitch_position);

    /**************************************************************
        * From the NUbots repo                                       *
        * File: ForwardKinematics.hpp                                *
        * Function: calculateHeadJointPosition                       *
        **************************************************************/
    Eigen::Affine3d Htx = Eigen::Affine3d::Identity();

    // From Kinematics Configuration
    const Eigen::Vector3d NECK_POS(-0.007, 0.0, 0.21);
    const double NECK_LENGTH = 0.048;

    // Translate to base of neck from origin
    Htx = Htx.translate(NECK_POS);
    // Rotate to face out of base of neck
    Htx = Htx.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
    // Rotate head in yaw axis
    Htx = Htx.rotate(Eigen::AngleAxisd(neck_yaw_sensor->getValue(), Eigen::Vector3d::UnitX()));
    // Translate to top of neck (i.e. next motor axle)
    Htx = Htx.translate(Eigen::Vector3d(NECK_LENGTH, 0.0, 0.0));

    // YAW
    // Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos
    // is top of neck (at hip pitch motor)

    // Rotate to face forward direction of neck
    Htx = Htx.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
    // Rotate pitch
    Htx = Htx.rotate(Eigen::AngleAxisd(head_pitch_sensor->getValue(), Eigen::Vector3d::UnitY()));
    // PITCH
    // Return basis pointing along camera vector (ie x is camera vector, z out of top of head). Pos at
    // camera position

    /**************************************************************
        * From the NUbots repo                                       *
        * Module: input/Camera                                       *
        * Files: Left.yaml and Right.yaml                            *
        **************************************************************/
    // clang-format off
    Eigen::Affine3d Hxl;
    Hxl.matrix() << 0.9997403694613228,   -0.016551656784825577, -0.01566002262642871,  0.08893612063588988,
                    0.01728764296000275,   0.9986932814319833,    0.04809227613146373,  0.03529909622045881,
                    0.014843552535558163, -0.04835051478781678,   0.9987201292902445,   0.0713674563254241,
                    0.0,                   0.0,                   0.0,                  1.0;
    Eigen::Affine3d Hxr;
    Hxr.matrix() << 0.9997403694613228,   -0.016551656784825577, -0.01566002262642871,  0.08893612063588988,
                    0.01728764296000275,   0.9986932814319833,    0.04809227613146373, -0.03529909622045881,
                    0.014843552535558163, -0.04835051478781678,   0.9987201292902445,   0.0713674563254241,
                    0.0,                   0.0,                   0.0,                  1.0;
    // clang-format on

    // Htw
    Eigen::Affine3d Htw = Eigen::Affine3d::Identity();
    Htw.linear()        = Eigen::AngleAxisd(rotation[3], Eigen::Vector3d(rotation[0], rotation[1], rotation[2]))
                        .toRotationMatrix()
                        .transpose();
    Htw.translation() =
        -Htw.linear() * Eigen::Vector3d(robot_position[0], robot_position[1], robot_position[2]);

    // Calculate the left and right camera to world transformation matrices
    Eigen::Affine3d Hwc_l = Htw.inverse() * Htx * Hxl;
    Eigen::Affine3d Hwc_r = Htw.inverse() * Htx * Hxr;

    //-----------SAVE DATA-----------//
    std::string count_padded = padLeft(count, 7);
    
    // Save mono images
    left_camera->saveImage("./data/data_mono/image" + count_padded + ".jpg", QUALITY);
    left_camera->saveRecognitionSegmentationImage("./data/data_mono/image" + count_padded + "_mask.png",
                                                QUALITY);
    // Save stereo images
    left_camera->saveImage("./data/data_stereo/image" + count_padded + "_L.jpg", QUALITY);
    right_camera->saveImage("./data/data_stereo/image" + count_padded + "_R.jpg", QUALITY);
    left_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + count_padded + "_L.png", QUALITY);
    right_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + count_padded + "_R.png",
                                                    QUALITY);

    // Prepare the mono lens data
    YAML::Emitter mono_lens;
    mono_lens << YAML::BeginMap;
    mono_lens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
    mono_lens << YAML::Key << "focal_length" << YAML::Value << focal_length_px;
    mono_lens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    mono_lens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    mono_lens << YAML::Key << "fov" << YAML::Value << diagonal_fov;
    mono_lens << YAML::Key << "Hoc" << YAML::Value;
    mono_lens << YAML::BeginSeq;
    mono_lens << YAML::Flow << std::vector({Hwc_l(0, 0), Hwc_l(0, 1), Hwc_l(0, 2), Hwc_l(0, 3)});
    mono_lens << YAML::Flow << std::vector({Hwc_l(1, 0), Hwc_l(1, 1), Hwc_l(1, 2), Hwc_l(1, 3)});
    mono_lens << YAML::Flow << std::vector({Hwc_l(2, 0), Hwc_l(2, 1), Hwc_l(2, 2), Hwc_l(2, 3)});
    mono_lens << YAML::Flow << std::vector({Hwc_l(3, 0), Hwc_l(3, 1), Hwc_l(3, 2), Hwc_l(3, 3)});
    mono_lens << YAML::EndSeq;
    mono_lens << YAML::EndMap;

    // Write the mono lens data
    std::ofstream ofs_mono("./data/data_mono/lens" + count_padded + ".yaml");
    ofs_mono << mono_lens.c_str();
    ofs_mono.close();

    // Prepare the stereo lens data
    YAML::Emitter stereo_lens;
    stereo_lens << YAML::BeginMap;

    stereo_lens << YAML::Key << "left";
    stereo_lens << YAML::BeginMap;
    stereo_lens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
    stereo_lens << YAML::Key << "focal_length" << YAML::Value << focal_length_px;
    stereo_lens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    stereo_lens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    stereo_lens << YAML::Key << "fov" << YAML::Value << diagonal_fov;
    stereo_lens << YAML::Key << "Hoc" << YAML::Value;
    stereo_lens << YAML::BeginSeq;
    stereo_lens << YAML::Flow << std::vector({Hwc_l(0, 0), Hwc_l(0, 1), Hwc_l(0, 2), Hwc_l(0, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_l(1, 0), Hwc_l(1, 1), Hwc_l(1, 2), Hwc_l(1, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_l(2, 0), Hwc_l(2, 1), Hwc_l(2, 2), Hwc_l(2, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_l(3, 0), Hwc_l(3, 1), Hwc_l(3, 2), Hwc_l(3, 3)});
    stereo_lens << YAML::EndSeq;
    stereo_lens << YAML::EndMap;

    stereo_lens << YAML::Key << "right";
    stereo_lens << YAML::BeginMap;
    stereo_lens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
    stereo_lens << YAML::Key << "focal_length" << YAML::Value << focal_length_px;
    stereo_lens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    stereo_lens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
    stereo_lens << YAML::Key << "fov" << YAML::Value << diagonal_fov;
    stereo_lens << YAML::Key << "Hoc" << YAML::Value;
    stereo_lens << YAML::BeginSeq;
    stereo_lens << YAML::Flow << std::vector({Hwc_r(0, 0), Hwc_r(0, 1), Hwc_r(0, 2), Hwc_r(0, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_r(1, 0), Hwc_r(1, 1), Hwc_r(1, 2), Hwc_r(1, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_r(2, 0), Hwc_r(2, 1), Hwc_r(2, 2), Hwc_r(2, 3)});
    stereo_lens << YAML::Flow << std::vector({Hwc_r(3, 0), Hwc_r(3, 1), Hwc_r(3, 2), Hwc_r(3, 3)});
    stereo_lens << YAML::EndSeq;
    stereo_lens << YAML::EndMap;
    stereo_lens << YAML::EndMap;

    // Write the stereo lens data
    std::ofstream ofs_stereo("./data/data_stereo/lens" + count_padded + ".yaml");
    ofs_stereo << stereo_lens.c_str();
    ofs_stereo.close();

    count++;
}

int main(int argc, char** argv) {

    // Make sure we have the command line arguments we need. At a minimum we should have the def argument
    // of the robot supervisor. Other arguments are def arguments of robots that should be controlled by
    // this supervisor
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <DEF>" << std::endl;
        return EXIT_FAILURE;
    }
    // Load def argument which will be used to identify the soccer field using the webots getFromDef function
    std::string fieldDef = argv[1];

    // Load def arguments of other robots in the field into a vector
    std::vector<webots::Node*> otherRobotsNodes;
    // Only one supervisor instance can exist in a single controller. As such the same one will be used
    // every time
    webots::Supervisor supervisor = webots::Supervisor();
    for (int i = 2; i < argc; i++) {
        // Load the Node of the robot by def argument and add to vector
        otherRobotsNodes.emplace_back(supervisor.getFromDef(argv[i]));
    }

    // Get the time step of the current world.
    int timeStep = int(supervisor.getBasicTimeStep());

    for (auto& robot : otherRobotsNodes) {
        if (robot->getTypeName() == "NUgus") {
            robot->getCamera("right_camera")->enable(timeStep);
            robot->getCamera("left_camera")->enable(timeStep);
            robot->getCamera("right_camera")->recognitionEnable(timeStep);
            robot->getCamera("left_camera")->recognitionEnable(timeStep);
            robot->getCamera("right_camera")->enableRecognitionSegmentation();
            robot->getCamera("left_camera")->enableRecognitionSegmentation();
            robot->getMotor("neck_yaw")->enable(timeStep);
            robot->getMotor("head_pitch")->enable(timeStep);
        }
    }

    // Create directories for saving data
    std::filesystem::create_directories("./data/data_mono");
    std::filesystem::create_directories("./data/data_stereo");

    // AxisAngle rotations will be read from a config file and saved here
    std::vector<std::array<double, 4>> rotations;
    double minDistance = 0.0;
    double zHeight     = 0.0;
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
    catch (...) {
        std::cerr << "Some other YAML error occurred.\n" << std::endl;
        return 3;
    }

    webots::Node* fieldNode = supervisor.getFromDef(fieldDef);
    const double xSize      = fieldNode->getField("xSize")->getSFFloat();
    const double ySize      = fieldNode->getField("ySize")->getSFFloat();


    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    // Generate distributions for random number systems in the following loop
    std::uniform_real_distribution<> xDistrib(-xSize * 0.5, xSize * 0.5);
    std::uniform_real_distribution<> yDistrib(-ySize * 0.5, ySize * 0.5);
    // The RNG for the index of the rotation selected this round
    std::uniform_int_distribution<size_t> rotDistrib(0, rotations.size() - 1);

    while (supervisor.step(timeStep) != -1) {
      
        // Declare a vector of positions that will be saved as they are randomly generated, to be later
        // applied to each robot
        std::vector<std::array<double, 3>> positions;
        // Loop through every robot
        for (auto& robots : otherRobotsNodes) {
            // Assume there is no collision
            bool collision = false;
            // newPos will be a "Proposed location" for a robot to teleport to
            std::array<double, 3> newPos{};
            do {
                collision = false;
                // Generate a new random location
                newPos[0] = xDistrib(gen);
                newPos[1] = yDistrib(gen);
                
                if (robots->getTypeName() == "RobocupSoccerBall") {
                    newPos[2] = 0.08;
                }
                else if (robots->getTypeName() == "Darwin-opHinge2Seg") {
                    newPos[2] = 0.24;
                }
                else {
                    newPos[2] = zHeight;
                }

                // Loop through the vector of existing proposed locations and see if the new one is going to
                // collide with any of them
                for (const auto& testPos : positions) {
                    const double distance =
                        std::sqrt(std::pow((newPos[1] - testPos[1]), 2) + std::pow((newPos[0] - testPos[0]), 2));
                    if (distance < minDistance) {
                        collision = true;
                    }
                }
                // Loop until a proposed location has been found that doesn't clash with the existing ones
            } while (collision);
            // Finally add the proposed location in as a confirmed position
            positions.emplace_back(newPos);
        }
        
        // Loop through every robot
        for (size_t i = 0; i < otherRobotsNodes.size(); i++) {
            // Grab translation field of the robot to modify
            // There will be a position for every robot in the positions vector
            otherRobotsNodes[i]->getField("translation")->setSFVec3f(positions[i].data());
            // Apply new rotation
            otherRobotsNodes[i]->getField("rotation")->setSFRotation(rotations[size_t(rotDistrib(gen))].data());

            // Reset physics to avoid robot tearing itself apart
            otherRobotsNodes[i]->resetPhysics();
        }
        
        // Collect vision data for all NUgus robots
        for (auto& robot : otherRobotsNodes) {
            if(robot->getTypeName() == "NUgus") {
                collect_vision_data(robot);
            }
        }

    };
    return 0;
}