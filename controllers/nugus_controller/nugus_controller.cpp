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
#include <webots/Device.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

extern "C" {
#include <sys/ioctl.h>
}

#include "messages.pb.h"

#include "utility/tcp.hpp"

using utility::tcp::close_socket;
using utility::tcp::create_socket_server;

using controller::nugus::AccelerometerMeasurement;
using controller::nugus::ActuatorRequests;
using controller::nugus::BumperMeasurement;
using controller::nugus::CameraExposure;
using controller::nugus::CameraMeasurement;
using controller::nugus::Force3DMeasurement;
using controller::nugus::ForceMeasurement;
using controller::nugus::GyroMeasurement;
using controller::nugus::MotorPID;
using controller::nugus::PositionSensorMeasurement;
using controller::nugus::SensorMeasurements;
using controller::nugus::Vector3;

class NUgus : public webots::Robot {
public:
    NUgus(const int& time_step_, const uint16_t& server_port_)
        : time_step(time_step_), server_port(server_port_), tcp_fd(create_socket_server(server_port_)) {
        send(tcp_fd, "Welcome", 8, 0);
    }
    ~NUgus() override {
        close_socket(tcp_fd);
    }
    // We want to prevent multiple NUguses connecting with the same port
    NUgus(NUgus& other) = delete;
    NUgus& operator=(NUgus& other) = delete;
    // Disable moving NUgus objects until we have tested that doing it doesn't break things
    NUgus(NUgus&& other) = delete;
    NUgus& operator=(NUgus&& other) = delete;

    void run() {
        int controller_time = 0;

        while (step(time_step) != -1) {
            // Don't bother doing anything unless we have an active TCP connection
            if (tcp_fd == -1) {
                std::cerr << "Error: Failed to start TCP server, retrying ..." << std::endl;
                tcp_fd = create_socket_server(server_port);
                send(tcp_fd, "Welcome", 8, 0);
                controller_time = 0;
                continue;
            }
            controller_time = int(controller_time + this->getBasicTimeStep());

            // Send data from the simulated robot hardware to the robot control software
            sendData(controller_time);

            // Check if we have received a message and deal with it if we have
            try {
                handleReceived();
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << std::endl;
                continue;
            }
        }
    }

    void parseActuatorRequests(const ActuatorRequests& actuatorRequests) {
        // For each motor in the message, get the motor and set the values for it
        for (int i = 0; i < actuatorRequests.motor_positions_size(); i++) {
            const MotorPID& motorPID = actuatorRequests.motor_pids(i);
            webots::Motor* motor     = this->getMotor(motorPID.name());
            if (motor != nullptr) {
                if (i < actuatorRequests.motor_positions_size()) {
                    motor->setPosition(actuatorRequests.motor_positions(i).position());
                }
                if (i < actuatorRequests.motor_velocities_size()) {
                    motor->setVelocity(actuatorRequests.motor_velocities(i).velocity());
                }
                if (i < actuatorRequests.motor_forces_size()) {
                    motor->setForce(actuatorRequests.motor_forces(i).force());
                }
                if (i < actuatorRequests.motor_torques_size()) {
                    motor->setTorque(actuatorRequests.motor_torques(i).torque());
                }
                if (i < actuatorRequests.motor_pids_size()) {
                    motor->setControlPID(motorPID.pid().x(), motorPID.pid().y(), motorPID.pid().z());
                }
            }
        }

        // For each camera in the message, set the exposure
        for (int i = 0; i < actuatorRequests.camera_exposures_size(); i++) {
            const CameraExposure& cameraExposure = actuatorRequests.camera_exposures(i);
            webots::Camera* camera               = this->getCamera(cameraExposure.name());
            if (camera != nullptr) {
                camera->setExposure(cameraExposure.exposure());
            }
        }

        // For each time step message sent, we enable that device if the value exists
        for (const auto& sensorTimeStep : actuatorRequests.sensor_time_steps()) {
            webots::Device* device = this->getDevice(sensorTimeStep.name());
            if (device != nullptr) {
                const uint32_t sensor_time_step = sensorTimeStep.timestep();
                // Add to our list of sensors if we have a time step, otherwise if we do not have a time step
                // remove it
                if (sensor_time_step > 0) {
                    sensors.insert(device);
                }
                else {
                    sensors.erase(device);
                }
                // Warn if the time step is non-zero and smaller than the basic time step
                // which shouldn't happen because smaller steps indicate the basic time step
                // is not "basic"
                const double basic_time_step = this->getBasicTimeStep();
                if (sensor_time_step != 0 && sensor_time_step < basic_time_step) {
                    std::cerr << "Time step for \"" + sensorTimeStep.name() + "\" should be greater or equal to "
                                     + std::to_string(basic_time_step) + ", ignoring "
                                     + std::to_string(sensor_time_step) + " value."
                              << std::endl;
                }
                // Warn if the time step is not a multiple of the basic time step
                // The time step should be some multiple of the basic time step
                else if (sensor_time_step % uint32_t(basic_time_step) != 0) {
                    std::cerr << "Time step for \"" + sensorTimeStep.name() + "\" should be a multiple of "
                                     + std::to_string(basic_time_step) + ", ignoring "
                                     + std::to_string(sensor_time_step) + " value."
                              << std::endl;
                }
                else {
                    // Given a valid time step, enable the corresponding device
                    switch (device->getNodeType()) {
                        case webots::Node::ACCELEROMETER: {
                            auto* accelerometer = reinterpret_cast<webots::Accelerometer*>(device);
                            accelerometer->enable(int(sensor_time_step));
                            break;
                        }
                        case webots::Node::CAMERA: {
                            auto* camera = reinterpret_cast<webots::Camera*>(device);
                            camera->enable(int(sensor_time_step));
                            break;
                        }
                        case webots::Node::GYRO: {
                            auto* gyro = reinterpret_cast<webots::Gyro*>(device);
                            gyro->enable(int(sensor_time_step));
                            break;
                        }
                        case webots::Node::POSITION_SENSOR: {
                            auto* positionSensor = reinterpret_cast<webots::PositionSensor*>(device);
                            positionSensor->enable(int(sensor_time_step));
                            break;
                        }
                        case webots::Node::TOUCH_SENSOR: {
                            auto* touchSensor = reinterpret_cast<webots::TouchSensor*>(device);
                            touchSensor->enable(int(sensor_time_step));
                            break;
                        }
                        default:
                            std::cerr << "Unable to enable unknown device WbNodeType: " << device->getNodeType()
                                      << std::endl;
                    }
                }
            }
        }
    }

    void handleReceived() {
        uint64_t available = 0;
        if (::ioctl(tcp_fd, FIONREAD, &available) < 0) {
            std::cerr << "Error querying for available data, " << strerror(errno) << std::endl;
            return;
        }

        const size_t old_size = buffer.size();
        buffer.resize(old_size + available);

        // Read data into our buffer and resize it to the new data we read
        ::read(tcp_fd, buffer.data() + old_size, available);

        // Function to read the payload length from the buffer
        auto read_length = [](const std::vector<uint8_t>& data) {
            return data.size() >= sizeof(uint32_t) ? ntohl(*reinterpret_cast<const uint32_t*>(data.data())) : 0u;
        };

        // So long as we have enough bytes to process an entire packet, process the packets
        for (uint32_t length = read_length(buffer); buffer.size() >= length + sizeof(length);
             length          = read_length(buffer)) {
            // Decode the protocol buffer and emit it as a message
            char* payload = reinterpret_cast<char*>(buffer.data()) + sizeof(length);

            // Parse message data
            ActuatorRequests actuatorRequests;
            if (!actuatorRequests.ParseFromArray(payload, int(length))) {
                throw std::runtime_error("Error: Failed to parse serialised message: " + std::string(strerror(errno)));
            }

            parseActuatorRequests(actuatorRequests);

            // Delete the packet we just read ready to read the next one
            buffer.erase(buffer.begin(), std::next(buffer.begin(), sizeof(length) + length));
        }
    }

    void sendData(int controller_time) {
        // Create the SensorMeasurements message
        auto sensorMeasurements = std::make_unique<SensorMeasurements>();

        sensorMeasurements->set_time(uint32_t(controller_time));
        struct timeval tp {};
        gettimeofday(&tp, nullptr);
        sensorMeasurements->set_real_time(uint64_t(tp.tv_sec * 1000 + tp.tv_usec / 1000));

        // Iterator over all the devices that have been enabled from any received ActuatorRequests messages
        for (auto* device : sensors) {
            switch (device->getNodeType()) {
                case webots::Node::ACCELEROMETER: {
                    auto* accelerometer = reinterpret_cast<webots::Accelerometer*>(device);
                    if ((time_step % accelerometer->getSamplingPeriod()) != 0) {
                        break;
                    }
                    AccelerometerMeasurement* measurement = sensorMeasurements->add_accelerometers();
                    measurement->set_name(accelerometer->getName());
                    const double* values = accelerometer->getValues();
                    Vector3* vector3     = measurement->mutable_value();
                    vector3->set_x(values[0]);
                    vector3->set_y(values[1]);
                    vector3->set_z(values[2]);
                    break;
                }
                case webots::Node::CAMERA: {
                    auto* camera = reinterpret_cast<webots::Camera*>(device);
                    if ((time_step % camera->getSamplingPeriod()) != 0) {
                        break;
                    }
                    CameraMeasurement* measurement = sensorMeasurements->add_cameras();
                    measurement->set_name(camera->getName());
                    measurement->set_width(uint32_t(camera->getWidth()));
                    measurement->set_height(uint32_t(camera->getHeight()));
                    measurement->set_quality(-1);  // raw image (JPEG compression not yet supported)
                    measurement->set_image(reinterpret_cast<const char*>(camera->getImage()));
                    break;
                }
                case webots::Node::GYRO: {
                    auto* gyro = reinterpret_cast<webots::Gyro*>(device);
                    if ((time_step % gyro->getSamplingPeriod()) != 0) {
                        break;
                    }
                    GyroMeasurement* measurement = sensorMeasurements->add_gyros();
                    measurement->set_name(gyro->getName());
                    const double* values = gyro->getValues();
                    Vector3* vector3     = measurement->mutable_value();
                    vector3->set_x(values[0]);
                    vector3->set_y(values[1]);
                    vector3->set_z(values[2]);
                    break;
                }
                case webots::Node::POSITION_SENSOR: {
                    auto* position_sensor = reinterpret_cast<webots::PositionSensor*>(device);
                    if ((time_step % position_sensor->getSamplingPeriod()) != 0) {
                        break;
                    }
                    PositionSensorMeasurement* measurement = sensorMeasurements->add_position_sensors();
                    measurement->set_name(position_sensor->getName());
                    measurement->set_value(position_sensor->getValue());
                    break;
                }
                case webots::Node::TOUCH_SENSOR: {
                    auto* touch_sensor = reinterpret_cast<webots::TouchSensor*>(device);
                    if ((time_step % touch_sensor->getSamplingPeriod()) != 0) {
                        break;
                    }
                    webots::TouchSensor::Type type = touch_sensor->getType();
                    // Find what type of touch sensor we have and set the right values for that type of touch sensor
                    switch (type) {
                        case webots::TouchSensor::BUMPER: {
                            BumperMeasurement* measurement = sensorMeasurements->add_bumpers();
                            measurement->set_name(touch_sensor->getName());
                            measurement->set_value(touch_sensor->getValue() == 1.0);
                            break;
                        }
                        case webots::TouchSensor::FORCE: {
                            ForceMeasurement* measurement = sensorMeasurements->add_forces();
                            measurement->set_name(touch_sensor->getName());
                            measurement->set_value(touch_sensor->getValue());
                            break;
                        }
                        case webots::TouchSensor::FORCE3D: {
                            Force3DMeasurement* measurement = sensorMeasurements->add_force3ds();
                            measurement->set_name(touch_sensor->getName());
                            const double* values = touch_sensor->getValues();
                            Vector3* vector3     = measurement->mutable_value();
                            vector3->set_x(values[0]);
                            vector3->set_y(values[1]);
                            vector3->set_z(values[2]);
                            break;
                        }
                    }
                    break;
                }
                default:
                    std::cerr << "Switch had no case. Unexpected WbNodeType: " << device->getNodeType() << std::endl;
                    break;
            }
        }

#ifndef NDEBUG  // set to print the created SensorMeasurements message for debugging
        std::cout << std::endl << std::endl << std::endl << "SensorMeasurements: " << std::endl;
        std::cout << "  sm.time: " << sensorMeasurements->time() << std::endl;
        std::cout << "  sm.real_time: " << sensorMeasurements->real_time() << std::endl;

        {
            std::cout << "  sm.messages: " << std::endl;
            int i = 0;
            for (auto message : sensorMeasurements->messages()) {
                std::cout << "    sm.messages[" << i << "]" << std::endl;
                std::cout << "      message_type: " << message.message_type() << std::endl;
                std::cout << "      text: " << message.text() << std::endl;
                i++;
            }
        }


        {
            std::cout << "  sm.accelerometers: " << std::endl;
            int i = 0;
            for (auto acc : sensorMeasurements->accelerometers()) {
                std::cout << "    sm.accelerometers[" << i << "]" << std::endl;
                std::cout << "      name: " << acc.name() << std::endl;
                std::cout << "      value: [" << acc.value().x() << ", " << acc.value().y() << ", " << acc.value().z()
                          << "]" << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.bumpers: " << std::endl;
            int i = 0;
            for (auto bumper : sensorMeasurements->bumpers()) {
                std::cout << "    sm.bumpers[" << i << "]" << std::endl;
                std::cout << "      name: " << bumper.name() << std::endl;
                std::cout << "      value: " << bumper.value() << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.cameras: " << std::endl;
            int i = 0;
            for (auto camera : sensorMeasurements->cameras()) {
                std::cout << "    sm.cameras[" << i << "]" << std::endl;
                std::cout << "      name: " << camera.name() << std::endl;
                std::cout << "      width: " << camera.width() << std::endl;
                std::cout << "      height: " << camera.height() << std::endl;
                std::cout << "      quality: " << camera.quality() << std::endl;
                std::cout << "      image (size): " << camera.image().size() << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.forces: " << std::endl;
            int i = 0;
            for (auto force : sensorMeasurements->forces()) {
                std::cout << "    sm.forces[" << i << "]" << std::endl;
                std::cout << "      name: " << force.name() << std::endl;
                std::cout << "      value: " << force.value() << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.force3ds: " << std::endl;
            int i = 0;
            for (auto force : sensorMeasurements->force3ds()) {
                std::cout << "    sm.force3ds[" << i << "]" << std::endl;
                std::cout << "      name: " << force.name() << std::endl;
                std::cout << "      value: [" << force.value().x() << ", " << force.value().y() << ", "
                          << force.value().z() << "]" << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.force6ds: " << std::endl;
            int i = 0;
            for (auto force : sensorMeasurements->force6ds()) {
                std::cout << "    sm.force6ds[" << i << "]" << std::endl;
                std::cout << "      name: " << force.name() << std::endl;
                std::cout << "      force: [" << force.force().x() << ", " << force.force().y() << ", "
                          << force.force().z() << "]" << std::endl;
                std::cout << "      torque: [" << force.torque().x() << ", " << force.force().y() << ", "
                          << force.force().z() << "]" << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.gyros: " << std::endl;
            int i = 0;
            for (auto gyro : sensorMeasurements->gyros()) {
                std::cout << "    sm.gyros[" << i << "]" << std::endl;
                std::cout << "      name: " << gyro.name() << std::endl;
                std::cout << "      value: [" << gyro.value().x() << ", " << gyro.value().y() << ", "
                          << gyro.value().z() << "]" << std::endl;
                i++;
            }
        }

        {
            std::cout << "  sm.position_sensors: " << std::endl;
            int i = 0;
            for (auto sensor : sensorMeasurements->position_sensors()) {
                std::cout << "    sm.position_sensors[" << i << "]" << std::endl;
                std::cout << "      name: " << sensor.name() << std::endl;
                std::cout << "      value: " << sensor.value() << std::endl;
                i++;
            }
        }
#endif

        // Try to send the message
        const std::string proto = sensorMeasurements->SerializeAsString();
        const uint32_t N        = htonl(uint32_t(proto.size()));
        if (send(tcp_fd, &N, sizeof(N), 0) < 0) {
            std::cerr << "Error: Failed to send message size over TCP connection: " << strerror(errno) << std::endl;
        }
        else if (send(tcp_fd, proto.data(), proto.size(), 0) < 0) {
            std::cerr << "Error: Failed to send data over TCP connection: " << strerror(errno) << std::endl;
        }
    }

private:
    /// Controller time step
    const int time_step;
    /// TCP server port
    const uint16_t server_port;
    /// File descriptor to use for the TCP connection
    int tcp_fd;
    /// Set of robot sensors
    std::set<webots::Device*> sensors;

    std::vector<uint8_t> buffer;
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
    int server_port = 0;
    try {
        server_port = std::stoi(argv[1]);
    }
    catch (...) {
        std::cerr << "Failed to convert server port number to an integer" << std::endl;
        return EXIT_FAILURE;
    }

    // Load in the simulation timestep from the command line and convert to an int
    int time_step = 0;
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
