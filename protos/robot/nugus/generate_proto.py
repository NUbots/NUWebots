import os
import xml.etree.ElementTree as ET
import re
import sys
if len(sys.argv) < 2:
    print("Please provide a path to the proto file.")
    sys.exit(1)
# Get the file path from command-line arguments
proto_file_path = sys.argv[1]
# Define the new constants
new_constants = '''field  SFVec3f     translation          0 0 0
  field  SFRotation  rotation             0 1 0 0
  field  SFString    name                 "nugus"            # Is `Robot.name`.
  # Find in "webots/projects/samples/contests/robocup/controllers/player"
  field  SFString    controller           "player"
  # Is `Robot.controllerArgs`.
  field  MFString    controllerArgs       []
  # Is `Robot.customData`.
  field  SFString    customData           ""
  # Is `Robot.supervisor`.
  field  SFBool      supervisor           FALSE
  # Is `Robot.synchronization`.
  field  SFBool      synchronization      TRUE
  # Is `Robot.selfCollision`.
  field  SFBool      selfCollision        TRUE
  # MOTOR PARAMETER: See section 2. of docs/Robot_Model_RoboCup_2021 for more information
  field  SFFloat     MX106-torque         10.00
  field  SFFloat     MX106-vel            5.76
  field  SFFloat     MX106-damping        1.23
  field  SFFloat     MX106-friction       2.55
  field  SFFloat     backlash             0.01
  field  SFFloat     gearMass             0.001
  field  SFFloat     DYNAMIXEL-RESOLUTION 0.0015
  # CAMERA PARAMETERS: See docs for more information
  # Approximates PI/2 radians (90 degrees)
  field SFFloat      fieldOfView            1.5707
  field SFInt32      cameraWidth            640              # 640 pixels
  field SFInt32      cameraHeight           480              # 480 pixels
  # 1.98mm, as from our real cameras (https://www.lensation.de/pdf/BF10M19828S118C.pdf)
  field SFFloat      cameraLensFocalLength  1.98
  # Not much noise on real cameras
  field SFFloat      cameraNoise            0.000000001
  # With over 100fps, real cameras do not have much motion blur
  field SFFloat      cameraMotionBlur       10
  field MFColor      recognitionColors      [0 0 1]
  # Used in the vision data collection tool
  field SFFloat      height                 0.51
'''

# Read the existing proto file
with open(proto_file_path, 'r') as file:
    filedata = file.read()
# Find the block to replace using a regex
constants_block = re.search(
    'PROTO nugus \[\n(.*?)\n\]\n{', filedata, re.DOTALL)
if constants_block is not None:
    # Replace the block
    filedata = filedata.replace(constants_block.group(1), new_constants)
    print("Replaced constants block.")
else:
    print("Constants block not found. Please check the structure of the proto file.")
# Replace all servo parameters with constants and change to HingeJointWithBacklash
filedata = filedata.replace("maxVelocity 20.0", "maxVelocity IS MX106-vel")
filedata = filedata.replace("maxTorque 1.0", "maxTorque IS MX106-torque")
filedata = filedata.replace(
    "HingeJoint {", "HingeJointWithBacklash {\n    backlash IS backlash\n    gearMass IS gearMass")
filedata = filedata.replace(
    "HingeJointParameters {", "HingeJointParameters {\n                    dampingConstant IS MX106-damping\n                    staticFriction IS MX106-friction")
filedata = filedata.replace(
    "PositionSensor {", "PositionSensor {\n                 resolution IS DYNAMIXEL-RESOLUTION")

# Add gyro and accelerometer to torso
filedata = filedata.replace('''selfCollision IS selfCollision
    children [''', '''selfCollision IS selfCollision
    children [
                            Transform {
                                # Accelerometer and Gyro are positioned 0.2m above torso base
                                translation 0.000000 0.000000 0.2
                                children [
                                #(See Section 1. of docs/Robot_Model_RoboCup_2021 for details on noise)
                                # Axis: x forward, y left, z up
                                # Units are [m/s^2]
                                # The acelerometer measures the reaction forces over 3 axes in [m/s^2], thus,
                                # at rest the accelerometer should read +9.81 [m/s^2] in the z axis.
                                # Range is -39.24 to 39.24 [m/s^2]
                                # Return value (2nd column of LUT) is offset by 100
                                Accelerometer {
                                    name "accelerometer"
                                    xAxis TRUE
                                    yAxis TRUE
                                    zAxis TRUE
                                    # Calculation used: (size of range)/4095
                                    resolution 0.01916
                                    lookupTable [
                                    -39.24 60.76 0.000704
                                    39.24  139.24 0.000307
                                    ]
                                }
                                # 'L3G4200D' Gyro
                                # (See Section 1. of docs/Robot_Model_RoboCup_2021 for details on noise)
                                # Axis: x forward, y left, z up
                                # Units are [rad/s]
                                # Range is -8.72665 to 8.72665 [rad/s]
                                # Return value (2nd column of LUT) is offset by 100
                                Gyro {
                                    name "gyroscope"
                                    xAxis TRUE
                                    yAxis TRUE
                                    zAxis TRUE
                                    # Calculation used: (size of range)/4095
                                    resolution 0.0042621
                                    lookupTable [
                                    -8.72665 91.238   0.0001151
                                    8.72665  108.762  0.000096541
                                    ]
                                }
                                ]
                            }
                        ''')

# Add cameras to head
filedata = filedata.replace(
    ''']
                    name "right_camera"''', '''
                      DEF right_camera Camera {
                        name "right_camera"
                        translation 0.0 0.0 0.0
                        rotation 0.0 0.0 1.0 0.0
                        # Set parameters from fields
                        fieldOfView IS fieldOfView
                        width IS cameraWidth
                        height IS cameraHeight
                        noise IS cameraNoise
                        motionBlur IS cameraMotionBlur
                        # We are using a rectilinear lens because there is no true spherical lens
                        spherical FALSE
                        # Allows the robot to detect objects in the world
                        recognition DEF recognition Recognition {
                          frameThickness 0       # Remove bounding boxes
                          segmentation TRUE      # Add segmentation
                        }
                        # Set the recognition parameters from the right camera
                        recognition USE recognition
                      }
                    ]
                    name "right_camera"''')
filedata = filedata.replace(
    ''']
                    name "left_camera"''', '''
                      DEF left_camera Camera {
                        name "left_camera"
                        translation 0.0 0.0 0.0
                        rotation 0.0 0.0 1.0 0.0
                        # Set parameters from fields
                        fieldOfView IS fieldOfView
                        width IS cameraWidth
                        height IS cameraHeight
                        noise IS cameraNoise
                        motionBlur IS cameraMotionBlur
                        # We are using a rectilinear lens because there is no true spherical lens
                        spherical FALSE
                        # Set the recognition parameters from the left camera
                        recognition USE recognition
                      }
                    ]
                    name "left_camera"''')

# Add touch sensors to feet
filedata = filedata.replace(''']
                                        name "right_foot"''',  '''
                                          # Define four touch sensors for the right foot
                                          # Back right touch sensor on right foot
                                          TouchSensor {
                                            translation -0.041 0.025 -0.061
                                            name "right_touch_sensor_br"
                                            type "bumper"
                                          }
                                          # Back right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.057 0.025 -0.061
                                            name "right_touch_sensor_bl"
                                            type "bumper"
                                          }
                                          # Front right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.057 -0.154 -0.061
                                            name "right_touch_sensor_fl"
                                            type "bumper"
                                          }
                                          # Front right touch sensor on right foot
                                          TouchSensor {
                                            translation -0.041 -0.154 -0.061
                                            name "right_touch_sensor_fr"
                                            type "bumper"
                                          }
                                        ]
                                        name "right_foot [foot]"''')

filedata = filedata.replace(''']
                                        name "left_foot"''',  '''
                                          # Define four touch sensors for the left foot
                                          # Back right touch sensor on left foot
                                          TouchSensor {
                                            translation -0.041 0.025 -0.061
                                            name "left_touch_sensor_br"
                                            type "bumper"
                                          }
                                          # Back left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.057 0.025 -0.061
                                            name "left_touch_sensor_bl"
                                            type "bumper"
                                          }
                                          # Front left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.057 -0.154 -0.061
                                            name "left_touch_sensor_fl"
                                            type "bumper"
                                          }
                                          # Front right touch sensor on left foot
                                          TouchSensor {
                                            translation -0.041 -0.154 -0.061
                                            name "left_touch_sensor_fr"
                                            type "bumper"
                                          }
                                        ]
                                        name "left_foot [foot]"''')
# Rename limbs
# Add colors for team stuff
# Write the file out again
with open(proto_file_path, 'w') as file:
    file.write(filedata)
