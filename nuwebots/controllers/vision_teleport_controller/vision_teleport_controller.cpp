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
 * Copyright 2022 NUbots <nubots@nubots.net>
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

// Create a string padded with 0's on the left and the given number on the right
// This is used to create file names
std::string pad_left(const int number, const int width) {
    std::stringstream ss;
    ss << std::setw(width) << std::setfill('0') << number;
    return ss.str();
}

// Randomly find robot positions that do not clash and return those positions
std::vector<std::array<double, 3>> find_robot_positions(const std::vector<webots::Node*>& robot_nodes,
                                                        std::uniform_real_distribution<> x_distrib,
                                                        std::uniform_real_distribution<> y_distrib,
                                                        std::mt19937 gen,
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

int main(int argc, char** argv) {
    // COMMAND LINE ARGUMENTS
    // Make sure we have the command line arguments we need. At a minimum we should have the def argument
    // of the robot supervisor. Other arguments are def arguments of robots that should be controlled by
    // this supervisor
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <DEF>" << std::endl;
        return EXIT_FAILURE;
    }
    // Load def argument which will be used to identify the soccer field using the webots getFromDef function
    const std::string field_def = argv[1];
    // Load def argument which will be used to identify the current robot using the webots getFromDef function
    const std::string self_def = argv[2];

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
    int time_step = int(supervisor.getBasicTimeStep());

    // GET AND ENABLE CAMERAS AND CALCULATE CAMERA PROPERTIES
    // Get the cameras, set their update time step and enable segmentation
    webots::Camera* left_camera  = supervisor.getCamera("left_camera");
    webots::Camera* right_camera = supervisor.getCamera("right_camera");
    left_camera->enable(time_step);
    right_camera->enable(time_step);
    left_camera->recognitionEnable(time_step);
    right_camera->recognitionEnable(time_step);
    left_camera->enableRecognitionSegmentation();
    right_camera->enableRecognitionSegmentation();

    // Calculate camera field of view
    const int camera_width       = left_camera->getWidth();
    const int camera_height      = left_camera->getHeight();
    const double camera_diagonal = std::sqrt(camera_width * camera_width + camera_height * camera_height);
    const double horizontal_fov  = left_camera->getFov();
    const double vertical_fov = 2 * std::atan(std::tan(horizontal_fov * 0.5) * (double(camera_height) / camera_width));
    const double focal_length_px = 0.5 * camera_height / std::tan(vertical_fov / 2.0);
    const double diagonal_fov    = 2 * std::atan(camera_diagonal / (2 * focal_length_px));

    // GET HANDLES TO NODES AND SERVOS
    // Get a handle to specific motors that will be moved later
    webots::Motor* neck_yaw   = supervisor.getMotor("neck_yaw");
    webots::Motor* head_pitch = supervisor.getMotor("head_pitch");
    webots::Motor* right_shoulder_pitch = supervisor.getMotor("right_shoulder_pitch [shoulder]");
    webots::Motor* left_shoulder_pitch  = supervisor.getMotor("left_shoulder_pitch [shoulder]");
    webots::Motor* left_elbow           = supervisor.getMotor("left_elbow_pitch");
    webots::Motor* right_elbow          = supervisor.getMotor("right_elbow_pitch");
    // Get sensors so that their joint angle can be retrieved when determining the camera to world transform
    // This transform is created and saved later when the data is saved
    webots::PositionSensor* neck_yaw_sensor   = supervisor.getPositionSensor("neck_yaw_sensor");
    webots::PositionSensor* head_pitch_sensor = supervisor.getPositionSensor("head_pitch_sensor");
    neck_yaw_sensor->enable(time_step);
    head_pitch_sensor->enable(time_step);

    // SET UP DATA DIRECTORIES
    // Create directories for saving data
    std::filesystem::create_directories("./data/data_mono");    // single camera dataset
    std::filesystem::create_directories("./data/data_stereo");  // two camera dataset
    int count = 0;                                              // counter for saving images

    // GET CONFIGURATION VALUES
    // Configuration values that will be loaded in from a yaml file
    double min_distance  = 0.0;  // minimum distance allowed between robots
    int image_quality    = 100;  // quality of the saved image [0,100]
    int file_name_length = 7;    // when saving data, e.g. 0000001.jpg has a length of 7 numbers identifying it

    // Load config file and handle errors
    try {
        YAML::Node config = YAML::LoadFile("vision_teleport_controller.yaml");
        min_distance      = config["min_distance"].as<double>();
        image_quality     = config["image_quality"].as<int>();
        file_name_length  = config["file_name_length"].as<int>();
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

    // GET RANDOM DISTRIBUTIONS FOR ROBOT LOCATION AND POSES
    // Grab the size of the field from the soccer field proto to restrict the bounds of
    // where the robots can teleport
    webots::Node* field_node = supervisor.getFromDef(field_def);
    const double x_size      = field_node->getField("xSize")->getSFFloat();
    const double y_size      = field_node->getField("ySize")->getSFFloat();

    // Generate random seed
    std::random_device rd;   // will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // standard mersenne_twister_engine seeded with rd()

    // Generate distributions for random number systems
    std::uniform_real_distribution<> x_distrib(-x_size * 0.5, x_size * 0.5);  // robot's x position on the field
    std::uniform_real_distribution<> y_distrib(-y_size * 0.5, y_size * 0.5);  // robot's y position on the field
    std::uniform_real_distribution<> rot_distrib(0, 2 * M_PI);                // robot's z-axis rotation
    std::uniform_real_distribution<> head_pitch_distrib(0.0, 1.0);            // head pitch joint angle
    std::uniform_real_distribution<> neck_yaw_distrib(-M_PI_2, M_PI_2);       // neck yaw joint angle
    std::uniform_real_distribution<> shoulder_distrib(0.0, M_PI_2);           // shoulder pitch joint angle
    std::uniform_real_distribution<> elbow_distrib(-M_PI_2, 0.0);             // elbow joint angle

    // MAIN UPDATE LOOP

    // In order to remove blurry images, images will only be saved on a disjointed
    // number of time_steps using a modulo operation
    int modulo_counter   = 0;
    constexpr int MODULO = 3;

    while (supervisor.step(time_step) != -1) {
        modulo_counter %= MODULO;  // prevents overflow error from counter

        // Reset physics of the robot running this controller to stabilise the image
        supervisor.getFromDef(self_def)->resetPhysics();

        // Move the robot on the iteration immediately after saving an image
        // Find valid random locations for the robots to teleport to, without collisions,
        // and move the robots to those locations with random rotations around the z-axis.
        // Move the head and arms of the main robot that is collecting data
        if (modulo_counter == 0) {
            // Get the positions to teleport the robots to
            const std::vector<std::array<double, 3>> positions =
                find_robot_positions(robot_nodes, x_distrib, y_distrib, gen, min_distance);

            // Loop through every robot and teleport it
            for (size_t i = 0; i < robot_nodes.size(); i++) {
                // Grab translation field of the robot to modify
                // There will be a position for every robot in the positions vector
                robot_nodes[i]->getField("translation")->setSFVec3f(positions[i].data());
                // Apply new rotation
                const std::array<double, 4> angle_axis_rot = {0.0, 0.0, 1.0, rot_distrib(gen)};
                robot_nodes[i]->getField("rotation")->setSFRotation(angle_axis_rot.data());

                // Reset physics to avoid robot tearing itself apart
                robot_nodes[i]->resetPhysics();
            }

            // Set random positions for the head and neck servos of the main robot
            neck_yaw->setPosition(neck_yaw_distrib(gen));
            head_pitch->setPosition(head_pitch_distrib(gen));
            right_shoulder_pitch->setPosition(shoulder_distrib(gen));
            left_shoulder_pitch->setPosition(shoulder_distrib(gen));
            right_elbow->setPosition(elbow_distrib(gen));
            left_elbow->setPosition(elbow_distrib(gen));

            // Update physics step enough that the motors will move
            supervisor.step(time_step * 20);
        }

        // Modulo 1 just advances the time

        // Collect vision data for the current environment
        else if (modulo_counter == 2) {
            // GET TRANSLATION AND ROTATION OF ROBOT
            webots::Field* robot_translation_field = supervisor.getFromDef(self_def)->getField("translation");
            const double* robot_position           = robot_translation_field->getSFVec3f();
            webots::Field* robot_rotation_field    = supervisor.getFromDef(self_def)->getField("rotation");
            const double* rotation                 = robot_rotation_field->getSFRotation();

            /**************************************************************
             * From the NUbots repo                                       *
             * File: ForwardKinematics.hpp                                *
             * Function: calculateHeadJointPosition                       *
             **************************************************************/
            // CALCULATE FORWARD KINEMATICS
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

            // Rotate to face forward direction of neck
            Htx = Htx.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
            // Rotate pitch
            Htx = Htx.rotate(Eigen::AngleAxisd(head_pitch_sensor->getValue(), Eigen::Vector3d::UnitY()));

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

            // Get world to torso transform
            Eigen::Affine3d Htw = Eigen::Affine3d::Identity();
            Htw.linear()        = Eigen::AngleAxisd(rotation[3], Eigen::Vector3d(rotation[0], rotation[1], rotation[2]))
                               .toRotationMatrix()
                               .transpose();
            Htw.translation() =
                -Htw.linear() * Eigen::Vector3d(robot_position[0], robot_position[1], robot_position[2]);

            // Calculate the left and right camera to world transformation matrices
            const Eigen::Affine3d Hwc_l = Htw.inverse() * Htx * Hxl;
            const Eigen::Affine3d Hwc_r = Htw.inverse() * Htx * Hxr;

            //-----------SAVE DATA-----------//
            const std::string count_padded = pad_left(count, file_name_length);

            // Save mono images
            left_camera->saveImage("./data/data_mono/image" + count_padded + ".jpg", image_quality);
            left_camera->saveRecognitionSegmentationImage("./data/data_mono/image" + count_padded + "_mask.png",
                                                          image_quality);
            // Save stereo images
            left_camera->saveImage("./data/data_stereo/image" + count_padded + "_L.jpg", image_quality);
            right_camera->saveImage("./data/data_stereo/image" + count_padded + "_R.jpg", image_quality);
            left_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + count_padded + "_L.png",
                                                          image_quality);
            right_camera->saveRecognitionSegmentationImage("./data/data_stereo/mask" + count_padded + "_R.png",
                                                           image_quality);

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

            // Increase the file naming count
            count++;
        }
        // Find the largest possible number for the file that the data can be saved as and restrict going above this
        if (count > (std::pow(10, file_name_length) - 1)) {
            std::cout << "Collected maximum number of data samples given file naming convention." << std::endl;
            return 0;   // not really an error, just the program's natural end
        }
        // Advance the counter to perform a different function on the next loop
        modulo_counter++;
    };
    return 0;
}
