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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
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
    // Load def argument which will be used to identify the current robot using the webots getFromDef function
    std::string selfDef = argv[2];

    // Load def arguments of other robots in the field into a vector
    std::vector<webots::Node*> robotNodes;
    // Only one supervisor instance can exist in a single controller. As such the same one will be used
    // every time
    webots::Supervisor supervisor = webots::Supervisor();
    for (int i = 2; i < argc; i++) {
        // Load the Node of the robot by def argument and add to vector
        robotNodes.emplace_back(supervisor.getFromDef(argv[i]));
    }

    // Get the time step of the current world.
    int timeStep = int(supervisor.getBasicTimeStep());

    // Get the cameras
    webots::Camera* leftCamera  = supervisor.getCamera("left_camera");
    webots::Camera* rightCamera = supervisor.getCamera("right_camera");
    leftCamera->enable(timeStep);
    rightCamera->enable(timeStep);
    leftCamera->recognitionEnable(timeStep);
    rightCamera->recognitionEnable(timeStep);
    leftCamera->enableRecognitionSegmentation();
    rightCamera->enableRecognitionSegmentation();

    // Calculate camera field of view
    int cameraWidth             = leftCamera->getWidth();
    int cameraHeight            = leftCamera->getHeight();
    const double cameraDiagonal = std::sqrt(cameraWidth * cameraWidth + cameraHeight * cameraHeight);
    const double horizontalFov  = leftCamera->getFov();
    const double verticalFov    = 2 * std::atan(std::tan(horizontalFov * 0.5) * (double(cameraHeight) / cameraWidth));
    const double focalLengthPx  = 0.5 * cameraHeight / std::tan(verticalFov / 2.0);
    const double diagonalFov    = 2 * std::atan(cameraDiagonal / (2 * focalLengthPx));

    //-----------GET HANDLES TO NODES AND SERVOS-----------//
    // Get a handle to our head and neck motors
    webots::Motor* neckYaw                  = supervisor.getMotor("neck_yaw");
    webots::Motor* headPitch                = supervisor.getMotor("head_pitch");
    webots::PositionSensor* neckYawSensor   = supervisor.getPositionSensor("neck_yaw_sensor");
    webots::PositionSensor* headPitchSensor = supervisor.getPositionSensor("head_pitch_sensor");


    // Create directories for saving data
    std::filesystem::create_directories("./data/data_mono");
    std::filesystem::create_directories("./data/data_stereo");
    // Counter for saving images
    int count = 0;

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

    // Grab the size of the field from the soccer field proto to restrict the bounds of
    // where the robots can teleport
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
    // Move servos in the range [-PI/2, PI/2]
    std::uniform_real_distribution<> servoDistrib(-M_PI_2, M_PI_2);

    // In order to remove blurry images, images will only be saved on a disjointed
    // number of timesteps using a modulo operation
    int moduloCounter    = 0;
    constexpr int MODULO = 3;

    while (supervisor.step(timeStep) != -1) {
        moduloCounter++;
        // Reset physics of the robot running this controller to stabilise the image
        supervisor.getFromDef(selfDef)->resetPhysics();

        // Move the robot on the iteration immediately after saving an image
        if (moduloCounter % MODULO == 1) {
            //-----------TRANSLATE ROBOTS-----------//

            // Declare a vector of positions that will be saved as they are randomly generated, to be later
            // applied to each robot
            std::vector<std::array<double, 3>> positions;
            // Loop through every robot
            for (auto& robots : robotNodes) {
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
                    else if (robots->getTypeName() == "RobotisOP3") {
                        newPos[2] = 0.29;
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
            for (size_t i = 0; i < robotNodes.size(); i++) {
                // Grab translation field of the robot to modify
                // There will be a position for every robot in the positions vector
                robotNodes[i]->getField("translation")->setSFVec3f(positions[i].data());
                // Apply new rotation
                robotNodes[i]->getField("rotation")->setSFRotation(rotations[size_t(rotDistrib(gen))].data());

                // Reset physics to avoid robot tearing itself apart
                robotNodes[i]->resetPhysics();
            }
            // Set random positions for the head and neck servos
            const double neckYawPosition   = servoDistrib(gen);
            const double headPitchPosition = servoDistrib(gen);
            neckYaw->setPosition(neckYawPosition);
            headPitch->setPosition(headPitchPosition);
        }

        // Run a number of iterations after the robot has moved equal to the value of modulo
        if (moduloCounter % MODULO == 0) {
            //----------GET TRANSLATION OF ROBOT------------//
            webots::Field* robotTranslationField = supervisor.getFromDef(selfDef)->getField("translation");
            const double* robot_position         = robotTranslationField->getSFVec3f();

            // ---------GET ROTATION OF ROBOT--------//
            webots::Field* robotRotationField = supervisor.getFromDef(selfDef)->getField("rotation");
            const double* rotation            = robotRotationField->getSFRotation();

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
            Htx = Htx.rotate(Eigen::AngleAxisd(neckYawSensor->getValue(), Eigen::Vector3d::UnitX()));
            // Translate to top of neck (i.e. next motor axle)
            Htx = Htx.translate(Eigen::Vector3d(NECK_LENGTH, 0.0, 0.0));

            // YAW
            // Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos
            // is top of neck (at hip pitch motor)

            // Rotate to face forward direction of neck
            Htx = Htx.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
            // Rotate pitch
            Htx = Htx.rotate(Eigen::AngleAxisd(headPitchSensor->getValue(), Eigen::Vector3d::UnitY()));
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
            std::string countPadded = padLeft(count, 7);

            // Save mono images
            leftCamera->saveImage("./data/data_mono/image" + countPadded + ".jpg", QUALITY);
            leftCamera->saveRecognitionSegmentationImage("./data/data_mono/image" + countPadded + "_mask.png", QUALITY);
            // Save stereo images
            leftCamera->saveImage("./data/data_stereo/image" + countPadded + "_L.jpg", QUALITY);
            rightCamera->saveImage("./data/data_stereo/image" + countPadded + "_R.jpg", QUALITY);
            leftCamera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + countPadded + "_L.png", QUALITY);
            rightCamera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + countPadded + "_R.png", QUALITY);

            // Prepare the mono lens data
            YAML::Emitter monoLens;
            monoLens << YAML::BeginMap;
            monoLens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            monoLens << YAML::Key << "focal_length" << YAML::Value << focalLengthPx;
            monoLens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            monoLens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            monoLens << YAML::Key << "fov" << YAML::Value << diagonalFov;
            monoLens << YAML::Key << "Hoc" << YAML::Value;
            monoLens << YAML::BeginSeq;
            monoLens << YAML::Flow << std::vector({Hwc_l(0, 0), Hwc_l(0, 1), Hwc_l(0, 2), Hwc_l(0, 3)});
            monoLens << YAML::Flow << std::vector({Hwc_l(1, 0), Hwc_l(1, 1), Hwc_l(1, 2), Hwc_l(1, 3)});
            monoLens << YAML::Flow << std::vector({Hwc_l(2, 0), Hwc_l(2, 1), Hwc_l(2, 2), Hwc_l(2, 3)});
            monoLens << YAML::Flow << std::vector({Hwc_l(3, 0), Hwc_l(3, 1), Hwc_l(3, 2), Hwc_l(3, 3)});
            monoLens << YAML::EndSeq;
            monoLens << YAML::EndMap;

            // Write the mono lens data
            std::ofstream ofsMono("./data/data_mono/lens" + countPadded + ".yaml");
            ofsMono << monoLens.c_str();
            ofsMono.close();

            // Prepare the stereo lens data
            YAML::Emitter stereoLens;
            stereoLens << YAML::BeginMap;

            stereoLens << YAML::Key << "left";
            stereoLens << YAML::BeginMap;
            stereoLens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            stereoLens << YAML::Key << "focal_length" << YAML::Value << focalLengthPx;
            stereoLens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            stereoLens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            stereoLens << YAML::Key << "fov" << YAML::Value << diagonalFov;
            stereoLens << YAML::Key << "Hoc" << YAML::Value;
            stereoLens << YAML::BeginSeq;
            stereoLens << YAML::Flow << std::vector({Hwc_l(0, 0), Hwc_l(0, 1), Hwc_l(0, 2), Hwc_l(0, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_l(1, 0), Hwc_l(1, 1), Hwc_l(1, 2), Hwc_l(1, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_l(2, 0), Hwc_l(2, 1), Hwc_l(2, 2), Hwc_l(2, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_l(3, 0), Hwc_l(3, 1), Hwc_l(3, 2), Hwc_l(3, 3)});
            stereoLens << YAML::EndSeq;
            stereoLens << YAML::EndMap;

            stereoLens << YAML::Key << "right";
            stereoLens << YAML::BeginMap;
            stereoLens << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            stereoLens << YAML::Key << "focal_length" << YAML::Value << focalLengthPx;
            stereoLens << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            stereoLens << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            stereoLens << YAML::Key << "fov" << YAML::Value << diagonalFov;
            stereoLens << YAML::Key << "Hoc" << YAML::Value;
            stereoLens << YAML::BeginSeq;
            stereoLens << YAML::Flow << std::vector({Hwc_r(0, 0), Hwc_r(0, 1), Hwc_r(0, 2), Hwc_r(0, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_r(1, 0), Hwc_r(1, 1), Hwc_r(1, 2), Hwc_r(1, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_r(2, 0), Hwc_r(2, 1), Hwc_r(2, 2), Hwc_r(2, 3)});
            stereoLens << YAML::Flow << std::vector({Hwc_r(3, 0), Hwc_r(3, 1), Hwc_r(3, 2), Hwc_r(3, 3)});
            stereoLens << YAML::EndSeq;
            stereoLens << YAML::EndMap;
            stereoLens << YAML::EndMap;

            // Write the stereo lens data
            std::ofstream ofsStereo("./data/data_stereo/lens" + countPadded + ".yaml");
            ofsStereo << stereoLens.c_str();
            ofsStereo.close();

            count++;
        }
    };
    return 0;
}
