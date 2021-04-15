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
 * Copyright 2021 NUbots <nubots@newcastle.edu.au>
 *
 * Portions of this code is taken from the official Webots RoboCup player controller
 * 2021 by Cyberbotics.
 */


#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include "messages.pb.h"

#include "utility/tcp.hpp"

#ifdef TURBOJPEG
#include <turbojpeg.h>
#else
#include <jpeglib.h>
#endif

using namespace utility::tcp;
using controller::nugus::ActuatorRequests;
using controller::nugus::CameraExposure;
using controller::nugus::CameraQuality;
using controller::nugus::MotorPID;
using controller::nugus::SensorMeasurements;
using controller::nugus::SensorTimeStep;

class NUgus : public webots::Robot {
public:
    NUgus(const int& time_step, const int& server_port)
        : time_step(time_step), server_port(server_port), tcp_fd(create_socket_server(server_port)) {
        send(tcp_fd, "Welcome", 8, 0);
    }
    ~NUgus() {
        close_socket(tcp_fd);
    }

    void run() {
        while (step(time_step) != -1) {
            // Don't bother doing anything unless we have an active TCP connection
            if (tcp_fd == -1) {
                std::cerr << "Error: Failed to start TCP server, retrying ..." << std::endl;
                tcp_fd = create_socket_server(server_port);
                send(tcp_fd, "Welcome", 8, 0);
                continue;
            }
            // Check if we have received a message and deal with it if we have
            try {
                handleReceived();
            }
            catch (const std::exception& e) {
                continue;
            }
            // Send data from the simulated robot hardware to the robot control software
            sendData();
        }
    }

    void handleReceived() {
        // Setup arguments for select call
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(tcp_fd, &rfds);
        timeval timeout = {0, 0};

        // Watch TCP file descriptor to see when it has input.
        // No wait - polling as fast as possible
        int num_ready = select(tcp_fd + 1, &rfds, nullptr, nullptr, &timeout);
        if (num_ready < 0) {
            std::cerr << "Error: Polling of TCP connection failed: " << strerror(errno) << std::endl;
            throw;
        }
        else if (num_ready > 0) {
            // Wire format
            // unit32_t Nn  message size in bytes. The bytes are in network byte order (big endian)
            // uint8_t * Nn  the message
            uint32_t Nn;
            if (recv(tcp_fd, &Nn, sizeof(Nn), 0) != sizeof(Nn)) {
                std::cerr << "Error: Failed to read message size from TCP connection: " << strerror(errno) << std::endl;
                throw;
            }

            // Covert to host endianness, which might be different to network endianness
            uint32_t Nh = ntohl(Nn);

            std::vector<uint8_t> data(Nh, 0);
            if (recv(tcp_fd, data.data(), Nh, 0) != Nh) {
                std::cerr << "Error: Failed to read message from TCP connection: " << strerror(errno) << std::endl;
                throw;
            }

            // Parse message data
            ActuatorRequests actuatorRequests;
            if (!actuatorRequests.ParseFromArray(data.data(), Nh)) {
                std::cerr << "Error: Failed to parse serialised message" << std::endl;
                throw;
            }

            //------PARSE ACTUATOR REQUESTS MESSAGE-----------
            // repeated MotorPosition motor_positions = 1;
            // repeated MotorVelocity motor_velocities = 2;
            // repeated MotorForce motor_forces = 3;
            // repeated MotorTorque motor_torques = 4;
            // repeated MotorPID motor_pids = 5;
            for (int i = 0; i < actuatorRequests.motor_positions_size(); i++) {
                const MotorPID motorPID = actuatorRequests.motor_pids(i);
                std::unique_ptr<webots::Motor> motor(this->getMotor(motorPID.name()));
                if (motor) {
                    motor->setPosition(actuatorRequests.motor_positions(i).position());
                    motor->setVelocity(actuatorRequests.motor_velocities(i).velocity());
                    motor->setForce(actuatorRequests.motor_forces(i).force());
                    motor->setTorque(actuatorRequests.motor_torques(i).torque());
                    motor->setControlPID(motorPID.pid().x(), motorPID.pid().y(), motorPID.pid().z());
                }

                // repeated CameraQuality camera_qualities = 7;
                // repeated CameraExposure camera_exposures = 8;
                // Quality not implemented yet
                // for (int i = 0; i < actuatorRequests.camera_qualities_size(); i++) {
                //     const CameraQuality cameraQuality = actuatorRequests.camera_qualities(i);
                //     std::unique_ptr<webots::Camera> camera(this->getCamera(cameraQuality.name()));
                //     if (camera) {
                //         camera->setQuality(cameraQuality.quality());
                //     }
                // }
                for (int i = 0; i < actuatorRequests.camera_exposures_size(); i++) {
                    const CameraExposure cameraExposure = actuatorRequests.camera_exposures(i);
                    std::unique_ptr<webots::Camera> camera(this->getCamera(cameraExposure.name()));
                    if (camera) {
                        camera->setExposure(cameraExposure.exposure());
                    }
                }

                // repeated SensorTimeStep sensor_time_steps = 6;
                // we need to enable the sensors after we sent the sensor value to avoid
                // sending values for disabled sensors.
                for (int i = 0; i < actuatorRequests.sensor_time_steps_size(); i++) {
                    const SensorTimeStep sensorTimeStep = actuatorRequests.sensor_time_steps(i);
                    std::unique_ptr<webots::Device> device(this->getDevice(sensorTimeStep.name()));
                    if (device) {
                        const int sensor_time_step = sensorTimeStep.timestep();
                        if (sensor_time_step) {
                            sensors.insert(device);
                        }                            
                        else {
                            sensors.erase(device);
                        }                            
                        bool valid_time_step = (sensor_time_step != 0 && sensor_time_step < basic_time_step) || (sensor_time_step % basic_time_step != 0);
                        if(valid_time_step) {
                            switch (device->getNodeType()) {
                                case webots::Node::ACCELEROMETER: {
                                    std::unique_ptr<webots::Accelerometer> accelerometer(this->getAccelerometer(sensorTimeStep.name()));
                                    accelerometer->enable(sensor_time_step);
                                    break;
                                }
                                case webots::Node::CAMERA: {
                                    std::unique_ptr<webots::Camera> camera(this->getCamera(sensorTimeStep.name()));
                                    camera->enable(sensor_time_step);
                                    break;
                                }
                                case webots::Node::GYRO: {
                                    std::unique_ptr<webots::Gyro> gyro(this->getGyro(sensorTimeStep.name()));
                                    gyro->enable(sensor_time_step);
                                    break;
                                }
                                case webots::Node::POSITION_SENSOR: {
                                    std::unique_ptr<webots::PositionSensor> positionSensor(this->getPositionSensor(sensorTimeStep.name()));
                                    positionSensor->enable(sensor_time_step);
                                    break;
                                }
                                case webots::Node::TOUCH_SENSOR: {
                                    std::unique_ptr<webots::TouchSensor> touchSensor(this->getTouchSensor(sensorTimeStep.name()));
                                    touchSensor->enable(sensor_time_step);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void sendData() {
        // Create the SensorMeasurements message
        auto msg = std::make_unique<SensorMeasurements>();

        //   uint32 time = 1;  // time stamp at which the measurements were performed expressed in [ms]
        // repeated Message messages = 2;
        // repeated AccelerometerMeasurement accelerometers = 3;
        // repeated BumperMeasurement bumpers = 4;
        // repeated CameraMeasurement cameras = 5;
        // repeated ForceMeasurement forces = 6;
        // repeated Force3DMeasurement force3ds = 7;
        // repeated Force6DMeasurement force6ds = 8;
        // repeated GyroMeasurement gyros = 9;
        // repeated PositionSensorMeasurement position_sensors = 10;

        for (std::set<std::unique_ptr<webots::Device>>::iterator it = sensors.begin(); it != sensors.end(); ++it) {
            webots::Accelerometer *accelerometer = dynamic_cast<webots::Accelerometer *>(*it);
            if (accelerometer) {
                if (controller_time % accelerometer->getSamplingPeriod()) {
                    continue;
                }
                AccelerometerMeasurement *measurement = sensorMeasurements.add_accelerometers();
                measurement->set_name(accelerometer->getName());
                const double *values = accelerometer->getValues();
                Vector3 *vector3 = measurement->mutable_value();
                vector3->set_x(values[0]);
                vector3->set_y(values[1]);
                vector3->set_z(values[2]);
                continue;
            }
            webots::Camera *camera = dynamic_cast<webots::Camera *>(*it);
            if (camera) {
                if (controller_time % camera->getSamplingPeriod()) {
                    continue;
                }                    
                CameraMeasurement *measurement = sensorMeasurements.add_cameras();
                measurement->set_name(camera->getName());
                measurement->set_width(camera->getWidth());
                measurement->set_height(camera->getHeight());
                measurement->set_quality(-1);  // raw image (JPEG compression not yet supported)
                measurement->set_image((const char *)camera->getImage());

                // testing JPEG compression (impacts the performance)
                unsigned char *buffer = NULL;
                long unsigned int bufferSize = 0;
                const unsigned char *image = camera->getImage();
                encode_jpeg(image, camera->getWidth(), camera->getHeight(), 95, &bufferSize, &buffer);
                free_jpeg(buffer);
                buffer = NULL;

                continue;
            }
            webots::Gyro *gyro = dynamic_cast<webots::Gyro *>(*it);
            if (gyro) {
                if (controller_time % gyro->getSamplingPeriod()) {
                    continue;
                }
                GyroMeasurement *measurement = sensorMeasurements.add_gyros();
                measurement->set_name(gyro->getName());
                const double *values = gyro->getValues();
                Vector3 *vector3 = measurement->mutable_value();
                vector3->set_x(values[0]);
                vector3->set_y(values[1]);
                vector3->set_z(values[2]);
                continue;
            }
            webots::PositionSensor *position_sensor = dynamic_cast<webots::PositionSensor *>(*it);
            if (position_sensor) {
                if (controller_time % position_sensor->getSamplingPeriod()) {
                    continue;
                }
                PositionSensorMeasurement *measurement = sensorMeasurements.add_position_sensors();
                measurement->set_name(position_sensor->getName());
                measurement->set_value(position_sensor->getValue());
                continue;
            }
            webots::TouchSensor *touch_sensor = dynamic_cast<webots::TouchSensor *>(*it);
            if (touch_sensor) {
                if (controller_time % touch_sensor->getSamplingPeriod()) {
                    continue;
                }                    
                webots::TouchSensor::Type type = touch_sensor->getType();
                switch (type) {
                    case webots::TouchSensor::BUMPER: {
                        BumperMeasurement *measurement = sensorMeasurements.add_bumpers();
                        measurement->set_name(touch_sensor->getName());
                        measurement->set_value(touch_sensor->getValue() == 1.0);
                        continue;
                    }
                    case webots::TouchSensor::FORCE: {
                        ForceMeasurement *measurement = sensorMeasurements.add_forces();
                        measurement->set_name(touch_sensor->getName());
                        measurement->set_value(touch_sensor->getValue());
                        continue;
                    }
                    case webots::TouchSensor::FORCE3D: {
                        Force3DMeasurement *measurement = sensorMeasurements.add_force3ds();
                        measurement->set_name(touch_sensor->getName());
                        const double *values = touch_sensor->getValues();
                        Vector3 *vector3 = measurement->mutable_value();
                        vector3->set_x(values[0]);
                        vector3->set_y(values[1]);
                        vector3->set_z(values[2]);
                        continue;
                    }
                }
            }
        }

        // Try to send the message
        uint32_t Nh = msg->ByteSizeLong();
        std::vector<uint8_t> data;
        data.resize(Nh);
        msg->SerializeToArray(data.data(), Nh);

        uint32_t Nn = htonl(Nh);

        if (send(tcp_fd, &Nn, sizeof(Nn), 0) < 0) {
            std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
        }
        else if (send(tcp_fd, data.data(), data.size(), 0) < 0) {
            std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
        }
    }

    static void encode_jpeg(const unsigned char* image,
                            int width,
                            int height,
                            int quality,
                            unsigned long* size,
                            unsigned char** buffer) {
#ifdef TURBOJPEG
        tjhandle compressor = tjInitCompress();
        tjCompress2(compressor, image, width, 0, height, TJPF_RGB, buffer, size, TJSAMP_444, quality, TJFLAG_FASTDCT);
        tjDestroy(compressor);
#else
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        JSAMPROW row_pointer[1];
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);
        cinfo.image_width      = width;
        cinfo.image_height     = height;
        cinfo.input_components = 3;
        cinfo.in_color_space   = JCS_RGB;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, quality, TRUE);
        jpeg_mem_dest(&cinfo, buffer, size);
        jpeg_start_compress(&cinfo, TRUE);
        while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = (unsigned char*) &image[cinfo.next_scanline * width * 3];
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }
        jpeg_finish_compress(&cinfo);
        jpeg_destroy_compress(&cinfo);
#endif
    }
    static void free_jpeg(unsigned char* buffer) {
#ifdef TURBOJPEG
        tjFree(buffer);
#else
        free(buffer);
#endif
    }

private:
    /// Controller time step
    const int time_step;
    /// TCP server port
    const int server_port;
    /// File descriptor to use for the TCP connection
    int tcp_fd;
    /// Set of robot sensors
    std::set<std::unique_ptr<webots::Device>> sensors;
};


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
    // Make sure we have the command line arguments we need
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <TCP PORT> <CONTROLLER_TIME_STEP>" << std::endl;
        return EXIT_FAILURE;
    }

    // Load in the TCP port number from the command line and convert to an int
    int server_port;
    try {
        server_port = std::stoi(argv[1]);
    }
    catch (...) {
        std::cerr << "Failed to convert server port number to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Load in the simulation timestep from the command line and convert to an int
    int time_step;
    try {
        time_step = std::stoi(argv[2]);
    }
    catch (...) {
        std::cerr << "Failed to convert simulation time step to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Create the Robot instance and initialise the TCP connection
    std::unique_ptr<NUgus> nugus = std::make_unique<NUgus>(time_step, server_port);

    // Run the robot controller
    nugus->run();

    return EXIT_SUCCESS;
}
