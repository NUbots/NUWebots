/*
*   Author: Benjamin Young
*   Purpose: Controller to test collision boxes
*       
*/

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <cmath>

#define TIME_STEP 32

using namespace webots;

int main() {

  // Declare Robot Object and Motor Objects
  Robot *robot = new Robot();
  
  // Right Shoulder
  Motor *motor_RSP = robot->getMotor("right_shoulder_pitch [shoulder]");
  Motor *motor_RSR = robot->getMotor("right_shoulder_roll");
  // Right Elbow
  Motor *motor_REP = robot->getMotor("right_elbow_pitch");
  
  // Left limb
  Motor *motor_LSP = robot->getMotor("left_shoulder_pitch [shoulder]");
  Motor *motor_LSR = robot->getMotor("left_shoulder_roll");
  // Left Elbow
  Motor *motor_LEP = robot->getMotor("left_elbow_pitch");
  
  // Right Hip
  Motor *motor_RHY = robot->getMotor("right_hip_yaw");
  Motor *motor_RHR = robot->getMotor("right_hip_roll [hip]");
  Motor *motor_RHP = robot->getMotor("right_hip_pitch");
  
  // Right Knee
  Motor *motor_RKP = robot->getMotor("right_knee_pitch");
  
  // Right Ankle
  Motor *motor_RAP = robot->getMotor("right_ankle_pitch");
  Motor *motor_RAR = robot->getMotor("right_ankle_roll");
  
  // Left Hip 
  Motor *motor_LHY = robot->getMotor("left_hip_yaw");
  Motor *motor_LHR = robot->getMotor("left_hip_roll [hip]");
  Motor *motor_LHP = robot->getMotor("left_hip_pitch");
  
  // Left Knee
  Motor *motor_LKP = robot->getMotor("left_knee_pitch");
  
  // Left Ankle
  Motor *motor_LAP = robot->getMotor("left_ankle_pitch");
  Motor *motor_LAR = robot->getMotor("left_ankle_roll");
  
  // Neck
  Motor *motor_NY = robot->getMotor("neck_yaw");
  Motor *motor_HP = robot->getMotor("head_pitch");

  double t = 0.0;         // Elapsed simulation time
  double pos = 0;         // Motor position
  
  //motor_RHY->setPosition(-2);
  motor_RHP->setPosition(-0.5);
  motor_LHP->setPosition(0.5);

  while (robot->step(TIME_STEP) != -1) {
  
    motor_RHP->setPosition(pos);
    motor_LHP->setPosition(-pos);
    pos = pos - 0.02;
    
    if(pos < -3.14)
      pos = 3.14;


    t += (double)TIME_STEP / 1000.0;
  }

  delete robot;
  return 0;
}
