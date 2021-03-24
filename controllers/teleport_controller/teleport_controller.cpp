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
#include <webots/Supervisor.hpp>
#include <iostream>
#include <time.h>  

/*
// Rotations matrix for 90 degree rotations
static const int rotationsSize = 4;
static const double rotations[rotationsSize][4] = {{1, 0, 0, 1.57081},
                                                  {0.577353, 0.577345, 0.577353, 2.0944},
                                                  {1.69526e-09, 0.707103, 0.707111, 3.14159},
                                                  {-0.577352, 0.577347, 0.577352, -2.09441}};

*/

// Rotations matrix for 45 degree rotations
static const int rotationsSize = 8;                                     
static const double rotations[rotationsSize][4] = {{1, 0, 0, 1.57081},
                                                  {0.867737, 0.343283, 0.359429, 1.75753},
                                                  {0.577353, 0.577345, 0.577353, 2.0944},
                                                  {0.286947, 0.661632, 0.692752, 2.60662},
                                                  {1.69526e-09, 0.707103, 0.707111, 3.14159},
                                                  {-0.286948, 0.661631, 0.692752, -2.60661},
                                                  {-0.577352, 0.577347, 0.577352, -2.09441},
                                                  {0.867739, -0.343277, -0.359429, 1.75753}};                               

constexpr int LOOK_AROUND = 54;
constexpr int CROUCH = 15;
constexpr int STAND_STILL = 1;


// This is the main program of the supervisor. It creates an instance of the Supervisor
// instance, launches its function(s) and destroys it at the end of the execution.

int main() {
    // create the Supervisor instance and assign it to a robot
    webots::Supervisor* supervisor = new webots::Supervisor();
    webots::Node* target           = supervisor->getFromDef("RED_1");
    
    // Get the time step of the current world.
    int timeStep = (int) supervisor->getBasicTimeStep();

    // Add the motion manager to make the robot stand up and look around
    managers::RobotisOp2MotionManager* mMotionManager = new managers::RobotisOp2MotionManager(supervisor);
    mMotionManager->playPage(54);

    // Generate random seed
    srand(time(NULL));

    while (supervisor->step(timeStep) != -1) {

        // Grab the current translation field of the robot to modify
        webots::Field* target_translation_field = target->getField("translation");
        // Convert the field to a vector to output to console
        const double* target_translation_vec = target_translation_field->getSFVec3f();

        // Output current location
        std::cout << "Location: " << std::endl;
        std::cout << "X: " << target_translation_vec[0];
        std::cout << " Y: " << target_translation_vec[1];
        std::cout << " Z: " << target_translation_vec[2] << std::endl;

        // Prepare new location 

        // 0, 0, 0 is centre of the playing field.
        // X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
        // from red
        // Y ranges from -8 - 8, the positive value on the red goal side
        // Z should be set to 3.2 to be on the ground
        // If the robot teleports into an existing object it may run into issues for that
        // image only, after resetPhysics is should return to a regular state
         

        double newPos[3];
        newPos[0] = 5.4 - (rand() % 1080) / 100;
        newPos[1] = 8 - (rand() % 1600) / 100;
        newPos[2] = 0.32;

        // Set new location
        target_translation_field->setSFVec3f(newPos);

        // Grab the current rotation field of the robot to modify
        webots::Field* target_rotation_field = target->getField("rotation");
        // Convert the field to a vector to output to console
        const double* target_rotation_vec = target_rotation_field->getSFRotation();

        // Loop 4 times, one for each direction to face
        for (int i = 0; i < rotationsSize; i++) {
            // Output current rotation
            std::cout << "Rotation: " << std::endl;
            std::cout << "X: " << target_rotation_vec[0];
            std::cout << " Y: " << target_rotation_vec[1];
            std::cout << " Z: " << target_rotation_vec[2];
            std::cout << " α: " << target_rotation_vec[3] << std::endl;

            // Prepare new rotation. These are saved in rotations matrix as the axis-angle
            // calculation is rough to calculate on the fly

            double newRot[4];
            newRot[0] = rotations[i][0];
            newRot[1] = rotations[i][1];
            newRot[2] = rotations[i][2];
            newRot[3] = rotations[i][3];

            // Set new rotations
            target_rotation_field->setSFRotation(newRot);

            // This line is very important. The robot blows itself up after a 1:05 minutes otherwise
            // Teleporting is strenuous work apparently
            target->resetPhysics();

            // The following performs a number of preset movements/routines that the robot can
            // carry out. Any of them can be commented out but leave at least one otherwise
            // there will be no delay between each step of this loop
            
            mMotionManager->playPage(LOOK_AROUND);
            mMotionManager->playPage(CROUCH);
            mMotionManager->playPage(STAND_STILL);
        }
    };

    delete supervisor;
    return 0;
}
