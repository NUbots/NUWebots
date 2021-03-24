// File:          teleport_controller.cpp
// Date:          23/03/2021
// Description:   Method to teleport robot around the playing field for the purposes of
//                    gathering training data
// Author:        Lachlan Court
// Modifications:


#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Supervisor.hpp>
#include <iostream>
#include <math.h>
#include <time.h> 

// All the webots classes are defined in the "webots" namespace
using namespace webots;

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


/* This is the main program of the supervisor. It creates an instance of the Supervisor
 *     instance, launches its function(s) and destroys it at the end of the execution.
 */
int main() {
    std::cout << "a" << std::endl;
    // create the Supervisor instance and assign it to a robot
    Supervisor* supervisor = new Supervisor();
    Node* target           = supervisor->getFromDef("RED_1");
    std::cout << "b" << std::endl;
    // Get the time step of the current world.
    int timeStep = (int) supervisor->getBasicTimeStep();

    std::cout << "c" << std::endl;


    /* The following block is intended to enable the Position Sensors. I must be doing something
     *     wrong here because it still comes up with an error, but if this is commented out it
     *     enables them automatically and just shows a warning on startup instead. I figure it
     *     works for now but might try to fix it later to get rid of the warning :)
     */

    // Add the motion manager to make the robot stand up and look around
    managers::RobotisOp2MotionManager* mMotionManager = new managers::RobotisOp2MotionManager(supervisor);
    mMotionManager->playPage(54);

    // Generate random seed
    srand(time(NULL));

    // Main loop:
    while (supervisor->step(timeStep) != -1) {

        // Grab the current translation field of the robot to modify
        Field* target_translation_field = target->getField("translation");
        // Convert the field to a vector to output to console
        const double* target_translation_vec = target_translation_field->getSFVec3f();

        // Output current location
        std::cout << "Location: " << std::endl;
        std::cout << "X: " << target_translation_vec[0];
        std::cout << " Y: " << target_translation_vec[1];
        std::cout << " Z: " << target_translation_vec[2] << std::endl;

        // Prepare new location

        /* 0, 0, 0 is centre of the playing field.
         *     X ranges from -5.4 - 5.4, the positive value on the left hand side when looking
         *     from red
         *     Y ranges from -8 - 8, the positive value on the red goal side
         *     Z should be set to 3.2 to be on the ground
         *     If the robot teleports into an existing object it may run into issues for that
         *     image only, after resetPhysics is should return to a regular state
         */

        double newPos[3];
        newPos[0] = 5.4 - (rand() % 1080) / 100;  // target_translation_vec[0] + 1;
        newPos[1] = 8 - (rand() % 1600) / 100;    // target_translation_vec[1] + 0;
        newPos[2] = 0.32;                         // target_translation_vec[2] + 0;

        // Set new location
        target_translation_field->setSFVec3f(newPos);

        // Grab the current rotation field of the robot to modify
        Field* target_rotation_field = target->getField("rotation");
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

            /* Prepare new rotation. These are saved in rotations matrix as the axis-angle
               calculation is rough to calculate on the fly*/

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

            /* Make robot stand up and look around. This preset is slow and I'm not super
             *     happy with it but it does the job for now. Below is a work in progress to
             *     manually move the robot motors to where we want them
             */
            mMotionManager->playPage(54);
            // Other presets, comment out as desired
            mMotionManager->playPage(15);
            mMotionManager->playPage(1);
        }
    };

    delete supervisor;
    return 0;
}
