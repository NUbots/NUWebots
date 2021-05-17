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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#include "yaml-cpp/yaml.h"

std::string padLeft(int number, int width) {
    std::stringstream ss;
    ss << std::setw(width) << std::setfill('0') << number;
    return ss.str();
}

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
    std::unique_ptr<webots::Supervisor> robot = std::make_unique<webots::Supervisor>();
    webots::Node* robot_def                   = robot->getFromDef(argv[1]);

    // Get the time step of the current world.
    int time_step = int(robot->getBasicTimeStep());

    // Get the cameras
    webots::Camera* left_camera  = robot->getCamera("left_camera");
    webots::Camera* right_camera = robot->getCamera("right_camera");
    right_camera->enable(time_step);
    left_camera->enable(time_step);
    right_camera->recognitionEnable(time_step);
    left_camera->recognitionEnable(time_step);
    right_camera->enableRecognitionSegmentation();
    left_camera->enableRecognitionSegmentation();

    // Calculate camera field of view
    int camera_width             = left_camera->getWidth();
    int camera_height            = left_camera->getHeight();
    const double camera_diagonal = std::sqrt(camera_width * camera_width + camera_height * camera_height);
    const double horizontal_fov  = left_camera->getFov();
    const double vertical_fov = 2 * std::atan(std::tan(horizontal_fov * 0.5) * (double(camera_height) / camera_width));
    const double focal_length_px = 0.5 * camera_height / std::tan(vertical_fov / 2.0);
    const double diagonal_fov    = 2 * std::atan(camera_diagonal / (2 * focal_length_px));

    // Create directories for saving data
    std::filesystem::create_directories("./data/data_mono");
    std::filesystem::create_directories("./data/data_stereo");

    //-----------SET RANDOM SEED-----------//
    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    // 0, 0, 0 is centre of the playing field.
    // X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
    // from red
    // Y ranges from -3.8 - 3.8, the positive value on the red goal side
    // Z should be set to 0.51 to be on the ground (and add a bit of noise)
    std::uniform_real_distribution<> xDistrib(-5.4, 5.4);
    std::uniform_real_distribution<> yDistrib(-3.8, 3.8);
    std::uniform_real_distribution<> zDistrib(0.46, 0.56);

    // Move servos in the range [-PI/2, PI/2]
    std::uniform_real_distribution<> servoDistrib(-M_PI * 0.5, M_PI * 0.5);

    //-----------GET HANDLES TO NODES AND SERVOS-----------//
    // Grab the current translation field of the robot to modify
    webots::Field* robot_translation_field = robot_def->getField("translation");

    // Grab the current rotation field of the robot to modify
    webots::Field* robot_rotation_field = robot_def->getField("rotation");

    // Get a handle to our head and neck motors
    webots::Motor* neck_yaw                   = robot->getMotor("neck_yaw");
    webots::Motor* head_pitch                 = robot->getMotor("head_pitch");
    webots::PositionSensor* neck_yaw_sensor   = robot->getPositionSensor("neck_yaw_sensor");
    webots::PositionSensor* head_pitch_sensor = robot->getPositionSensor("head_pitch_sensor");
    neck_yaw_sensor->enable(time_step);
    head_pitch_sensor->enable(time_step);

    //--------MAIN CONTROL LOOP------------//
    while (robot->step(time_step) != -1) {
        //----------SET TRANSLATION OF ROBOT------------//

        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state
        const std::array<double, 3> robot_position = {xDistrib(gen), yDistrib(gen), zDistrib(gen)};

        // Set new location
        robot_translation_field->setSFVec3f(robot_position.data());

        // ---------LOOP OVER ROTATIONS--------//
        for (const std::array<double, 4>& rotation : rotations) {

            // Set random positions for the head and neck servos
            neck_yaw->setPosition(servoDistrib(gen));
            head_pitch->setPosition(servoDistrib(gen));

            //-----------SET ROTATION----------//
            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly
            // Apply new rotation and reset physics to avoid robot tearing itself apart
            robot_rotation_field->setSFRotation(rotation.data());  // Rotation is interpreted as [rx, ry, rz, \alpha]
            robot_def->resetPhysics();

            // The step is needed to make the position and rotation changes take effect
            if (robot->step(time_step) == -1) {
                break;
            }

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

            // Save stereo images
            left_camera->saveImage("./data/data_stereo/image" + count_padded + "_L.jpg", QUALITY);
            right_camera->saveImage("./data/data_stereo/image" + count_padded + "_R.jpg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_stereo/image" + count_padded + "_L_mask.png",
                                                          QUALITY);
            right_camera->saveRecognitionSegmentationImage("./data/data_stereo/image" + count_padded + "_R_mask.png",
                                                           QUALITY);

            // Save mono images
            left_camera->saveImage("./data/data_mono/image" + count_padded + ".jpg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_mono/image" + count_padded + "_mask.png",
                                                          QUALITY);

            // For some reason the first iteration has the robot in the wrong position
            if (count == 0) {
                count++;
                continue;
            }

            // Prepare the lens data
            YAML::Emitter lensYaml;  // create the node
            lensYaml << YAML::BeginMap;
            lensYaml << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            lensYaml << YAML::Key << "focal_length" << YAML::Value << focal_length_px;
            lensYaml << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "fov" << YAML::Value << diagonal_fov;
            lensYaml << YAML::Key << "Hoc" << YAML::Value;
            lensYaml << YAML::BeginSeq;
            lensYaml << YAML::Flow << std::vector({Hwc_l(0, 0), Hwc_l(0, 1), Hwc_l(0, 2), Hwc_l(0, 3)});
            lensYaml << YAML::Flow << std::vector({Hwc_l(1, 0), Hwc_l(1, 1), Hwc_l(1, 2), Hwc_l(1, 3)});
            lensYaml << YAML::Flow << std::vector({Hwc_l(2, 0), Hwc_l(2, 1), Hwc_l(2, 2), Hwc_l(2, 3)});
            lensYaml << YAML::Flow << std::vector({Hwc_l(3, 0), Hwc_l(3, 1), Hwc_l(3, 2), Hwc_l(3, 3)});
            lensYaml << YAML::EndSeq;
            lensYaml << YAML::EndMap;

            // Write the stereo lens data
            std::ofstream stereoLensFile("./data/data_stereo/lens" + count_padded + ".yaml");
            stereoLensFile << lensYaml.c_str();
            stereoLensFile.close();

            // Write the mono lens data
            std::ofstream monoLensFile("./data/data_mono/lens" + count_padded + ".yaml");
            monoLensFile << lensYaml.c_str();
            monoLensFile.close();

            count++;
        }
    }
    return 0;
}
