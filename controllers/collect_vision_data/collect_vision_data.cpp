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
#include <sstream>
#include <iomanip>

#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>

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
    std::filesystem::create_directories("./data/data_mono");
    std::filesystem::create_directories("./data/data_stereo");

    //-----------SET RANDOM SEED-----------//
    // Generate random seed
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> xDistrib(0, 1080);
    std::uniform_int_distribution<> yDistrib(0, 760);
    std::uniform_int_distribution<> zDistrib(-5, 5);

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
        newPos[0] = 0.0; // 5.4 - xDistrib(gen) / 100;
        newPos[1] = 0.0; // 3.8 - yDistrib(gen) / 100;
        newPos[2] = 0.51; // - zDistrib(gen) / 100;

        // Set new location
        robot_translation_field.setSFVec3f(newPos.data());
        supervisor.step(time_step);

        // const double* values = robot_translation_field.getSFVec3f();
        // std::cout << values[0] << ", " << values[1] << ", " << values[2] << std::endl;
        // std::cout << newPos[0] << ", " << newPos[1] << ", " << newPos[2] << std::endl;

        // Grab the current rotation field of the robot to modify
        webots::Field& robot_rotation_field = *(robot.getField("rotation"));
        // Convert the field to a vector to output to console
        const double* robot_rotation_vec = robot_rotation_field.getSFRotation();

        // ---------LOOP OVER ROTATIONS--------//
        for (const std::array<double, 4>& rotation : rotations) {
            const std::array<double, 4> rotationX = rotation; // { 1.000000, 0.000000, 0.0, 0 };

            //-----------SET ROTATION----------//
            // Prepare new rotation. These are saved in rotations vector as the axis-angle
            // calculation is rough to calculate on the fly
            // Apply new rotation and reset physics to avoid robot tearing itself apart
            robot_rotation_field.setSFRotation(rotationX.data());  // Rotation is interpreted as [rx, ry, rz, \alpha]
            robot.resetPhysics();

            // TODO(KipHamiltons) verify this is the right rotation - as in, verify is Roc, not Rco, and that the encoding of xyzw is the same order here
            // Get WORLD TO TORSO
            // World to torso transformation
            Eigen::Affine3d Htw;
            Htw.linear() = Eigen::AngleAxisd(rotationX[3], Eigen::Vector3d(rotationX[0], rotationX[1], rotationX[2])).toRotationMatrix();
            Htw.translation() = Htw.rotation() * -Eigen::Vector3d(newPos.data());

            std::cout << "Htw " << std::endl << Htw.matrix() << std::endl << std::endl;

            // Get TORSO TO NECK
            // relative to torso, torso to neck
            const double* rNTt = robot.getFromProtoDef("neck_solid")->getField("translation")->getSFVec3f();
            // Rtn, torso to neck - Head Yaw?
            const double* Rnt = robot.getFromProtoDef("neck_solid")->getField("rotation")->getSFRotation();  // Rotation is interpreted as [rx, ry, rz, \alpha]

            // Homogenous transformation of the neck to the robot's torso - Matrix from torso to head yaw
            Eigen::Affine3d Hnt;
            Hnt.linear() = Eigen::AngleAxisd(Rnt[3], Eigen::Vector3d(Rnt[0], Rnt[1], Rnt[2])).toRotationMatrix();
            Hnt.translation() = Hnt.rotation() * -Eigen::Vector3d(rNTt[0], rNTt[1], rNTt[2]);

            std::cout << "Hnt " << std::endl << Hnt.matrix() << std::endl << std::endl;


            // Get NECK TO TRANSFORM - Head yaw to head pitch
            // relative to torso, torso to neck
            const double* rXNn = robot.getFromProtoDef("neck_transform")->getField("translation")->getSFVec3f();
            // Rtn, torso to neck
            const double* Rxn = robot.getFromProtoDef("neck_transform")->getField("rotation")->getSFRotation();  // Rotation is interpreted as [rx, ry, rz, \alpha]

            // Homogenous transformation of the neck to the robot's torso - Matrix from head yaw to head pitch
            Eigen::Affine3d Hxn;
            Hxn.linear() = Eigen::AngleAxisd(Rxn[3], Eigen::Vector3d(Rxn[0], Rxn[1], Rxn[2])).toRotationMatrix();
            Hxn.translation() = Hxn.rotation() * -Eigen::Vector3d(rXNn[0], rXNn[1], rXNn[2]);

            std::cout << "Hxn " << std::endl << Hxn.matrix() << std::endl << std::endl;

            // Get TRANSFORM TO CAMERA
            // const double* rCXn = robot.getFromProtoDef("right_camera")->getField("translation")->getSFVec3f();
            // const double* Rcx = robot.getFromProtoDef("right_camera")->getField("rotation")->getSFRotation();  // Rotation is interpreted as [rx, ry, rz, \alpha]
            
            Eigen::Affine3d Hcx;

            Hcx.linear() = Eigen::AngleAxisd(2* M_PI / 180, Eigen::Vector3d(0,1,0)).toRotationMatrix();
            Hcx.translation() = Hcx.rotation() * -Eigen::Vector3d(0.069952, 0.033795, 0.06488);

            std::cout << "Hcx " << std::endl << Hcx.matrix() << std::endl << std::endl;

            // Hcx.linear() = Eigen::AngleAxisd(Rcx[3], Eigen::Vector3d(Rcx[0], Rcx[1], Rcx[2])).toRotationMatrix();
            // Hcx.translation() = Hcx.rotation() * -Eigen::Vector3d(rCXn[0], rCXn[1], rCXn[2]);

            std::cout << "(Hnt * Htw) " << std::endl << (Hnt * Htw).matrix() << std::endl << std::endl;
            std::cout << "(Hxn * (Hnt * Htw)) " << std::endl << (Hxn * (Hnt * Htw)).matrix() << std::endl << std::endl;
            std::cout << "(Hcx * (Hxn * (Hnt * Htw))) " << std::endl << (Hcx * (Hxn * (Hnt * Htw))).matrix() << std::endl << std::endl;

            Eigen::Affine3d Hcw = (Hcx * (Hxn * (Hnt * Htw)));
            Eigen::Affine3d Hwc = Hcw.inverse();

            // This fixes roll being inverted, but we're not sure why
            // Hwc.linear() = Hwc.rotation().transpose();


            std::cout << "Hcw " << std::endl << Hcw.matrix() << std::endl << std::endl;
            std::cout << "Hwc " << std::endl << Hwc.matrix() << std::endl << std::endl;

            // Hoc.linear() = Roc.matrix();
            // // TODO(YsobelSims) should this vector be negated?? this might be rWCc or rCWc or some other thing kip didn't think of
            // Hoc.translation() = Eigen::Vector3d(newPos.data());

            //-----------SAVE DATA-----------//
            std::string count_padded = padLeft(count, 7);

            // Save stereo images
            left_camera->saveImage("./data/data_stereo/image" + count_padded + "_L.jpg", QUALITY);
            right_camera->saveImage("./data/data_stereo/image" + count_padded + "_R.jpg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_stereo/image" + count_padded + "_L_mask.png", QUALITY);
            right_camera->saveRecognitionSegmentationImage("./data/data_stereo/image" + count_padded + "_R_mask.png", QUALITY);

            // Save mono images
            left_camera->saveImage("./data/data_mono/image" + count_padded + ".jpg", QUALITY);
            left_camera->saveRecognitionSegmentationImage("./data/data_mono/image" + count_padded + "_mask.png", QUALITY);

            // For some reason the first iteration has the robot in the wrong position
            if (count == 0) {
                count++;
                continue;
            }

            // Prepare the lens data
            YAML::Emitter lensYaml;  // create the node
            lensYaml << YAML::BeginMap;
            lensYaml << YAML::Key << "projection" << YAML::Value << "RECTILINEAR";
            lensYaml << YAML::Key << "focal_length" << YAML::Value << 420;
            lensYaml << YAML::Key << "centre" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "k" << YAML::Flow << YAML::BeginSeq << 0 << 0 << YAML::EndSeq;
            lensYaml << YAML::Key << "fov" << YAML::Value << left_camera->getFov();
            lensYaml << YAML::Key << "Hoc" << YAML::Value;
            lensYaml << YAML::BeginSeq;
            lensYaml << YAML::Flow << std::vector({Hwc(0, 0), Hwc(0, 1), Hwc(0, 2), Hwc(0,3)});
            lensYaml << YAML::Flow << std::vector({Hwc(1, 0), Hwc(1, 1), Hwc(1, 2), Hwc(1,3)});
            lensYaml << YAML::Flow << std::vector({Hwc(2, 0), Hwc(2, 1), Hwc(2, 2), Hwc(2,3)});
            lensYaml << YAML::Flow << YAML::BeginSeq << 0 << 0 << 0 << 1 << YAML::EndSeq;
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
            supervisor.step(time_step);
        }
    }
    return 0;
}
