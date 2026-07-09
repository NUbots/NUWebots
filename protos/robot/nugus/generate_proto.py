import numpy as np
from scipy.spatial import ConvexHull
import os
import xml.etree.ElementTree as ET
import pymeshlab
import re
import sys
from urdf2webots.importer import convertUrdfFile

# Parse URDF file
tree = ET.parse('robot.urdf')
root = tree.getroot()

# Get all mesh filenames from URDF
mesh_files = []
collision_elements = []
for mesh in root.findall(".//mesh"):
    filename = mesh.attrib['filename']

    # Remove 'package:///' prefix
    clean_filename = filename.replace('package:///', '')
    mesh.set('filename', clean_filename)

    mesh_files.append(clean_filename)
    if clean_filename.endswith('_collision.stl'):
        collision_elements.append(mesh)

# Simplify each STL file referenced in URDF
for file in mesh_files:
    # Load mesh
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(file)
    # Simplify mesh
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=21845)
    # Save simplified mesh
    ms.save_current_mesh(file)

# Find all collision STL files
collision_files = [
    file for file in mesh_files if file.endswith('_collision.stl')]

# Process collision elements
for collision in root.findall(".//collision"):
    mesh_element = collision.find(".//mesh")

    # Remove mesh element from geometry
    geometry_element = collision.find(".//geometry")

    if mesh_element is not None:
        filename = mesh_element.attrib['filename'].replace('package:///', '')

        # Load mesh
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(filename)

        # Compute the optimal bounding box
        bounding_box = ms.current_mesh().bounding_box()

        # Get min and max coordinates
        min_x = bounding_box.min()[0]
        max_x = bounding_box.max()[0]
        min_y = bounding_box.min()[1]
        max_y = bounding_box.max()[1]
        min_z = bounding_box.min()[2]
        max_z = bounding_box.max()[2]

        # Calculate bounding box dimensions
        x_size = max_x - min_x
        y_size = max_y - min_y
        z_size = max_z - min_z

        # Calculate center of bounding box
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center_z = (min_z + max_z) / 2

        # Create or update the origin element
        origin_element = collision.find('./origin')
        if origin_element is None:
            origin_element = ET.SubElement(collision, 'origin')
        origin_element.attrib['xyz'] = f'{center_x} {center_y} {center_z}'

        # Create box element
        box_element = ET.Element('box', size=f'{x_size} {y_size} {z_size}')

        # Find the parent geometry element
        if geometry_element is not None:
            # Remove mesh element from the geometry
            geometry_element.remove(mesh_element)

            # Add the box element to the geometry
            geometry_element.append(box_element)

# Save modified URDF
tree.write('robot.urdf')

# Delete all STL files not in URDF
for file in os.listdir('.'):
    if file.endswith('.stl') and file not in mesh_files:
        os.remove(file)

# Delete all STL files used for collision
for file in os.listdir('.'):
    if file.endswith('_collision.stl'):
        os.remove(file)

# Delete all .part files
for file in os.listdir('.'):
    if file.endswith('.part'):
        os.remove(file)


urdf_file_path = "robot.urdf"
proto_file_path = "nugus.proto"

# Convert URDF to PROTO using urdf2webots library
convertUrdfFile(input=urdf_file_path, output=proto_file_path,
                boxCollision=False, normal=False)

# Read the existing proto file
with open(proto_file_path, 'r') as file:
    filedata = file.read()

# Replace constants block
filedata = filedata.replace('''PROTO nugus [
  field  SFVec3f     translation     0 0 0
  field  SFRotation  rotation        0 0 1 0
  field  SFString    name            "nugus"  # Is `Robot.name`.
  field  SFString    controller      "void"   # Is `Robot.controller`.
  field  MFString    controllerArgs  []       # Is `Robot.controllerArgs`.
  field  SFString    customData      ""       # Is `Robot.customData`.
  field  SFBool      supervisor      FALSE    # Is `Robot.supervisor`.
  field  SFBool      synchronization TRUE     # Is `Robot.synchronization`.
  field  SFBool      selfCollision   FALSE    # Is `Robot.selfCollision`.
]''', '''
EXTERNPROTO "JerseyBack.proto"
EXTERNPROTO "JerseyFront.proto"
PROTO nugus 
[
    field  SFVec3f     translation          0 0 0
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
    field  SFBool      selfCollision        FALSE
    # MOTOR PARAMETERS
    # Each Dynamixel model is described by its electrical constants at a 14.8 V bus.
    #
    # useVoltageModel (default FALSE) selects how each motor's actuator limits are set:
    #   FALSE -> fixed nominal limits (the literals injected below); voltage/current/K are
    #            inert documentation. This is the plain, backwards-compatible behaviour.
    #   TRUE  -> limits are DERIVED from the electrical fields, so voltage and current
    #            actually drive the model and can be overridden per robot instance:
    #                maxTorque   [N.m]   = K * current    (stall torque = K * stall current)
    #                maxVelocity [rad/s] = voltage / K    (no-load speed = V / back-EMF const)
    # Both paths yield the same nominal numbers (the electrical fields are calibrated to
    # reproduce the fixed limits); the toggle only changes whether they are live or fixed.
    # NB: the nominal maxVelocity values (see NOMINAL below) must match the nubots
    # controller's max_velocity_* config (module/platform/Webots/data/config/Webots.yaml),
    # which caps commanded servo speed -- otherwise the walk fights the actuator limit.
    #
    # K is the joint-side torque/back-EMF constant [N.m/A] (gearbox lumped in) and current
    # is the stall current V/R, both from system identification (servo_id_results.json)
    # unless noted.
    field  SFBool      useVoltageModel      FALSE
    # MX64: shoulder pitch/roll, elbow, neck yaw, head pitch (8 total)
    #   K/current reproduce the 2021 doc's datasheet torque (7.30 N.m) and no-load speed
    #   (8.17 rad/s, docs/Robot_Model_RoboCup_2021) -- the sys-id fit for these joints was
    #   either poor (shoulder roll, elbow, head pan/tilt) or, for shoulder pitch, implied an
    #   implausible no-load speed (~3x datasheet) off only 2 well-fit joints, too thin a
    #   sample to trust over the bench-derived datasheet numbers.
    # Field names use underscores, not hyphens: the template engine looks them up in the
    # Lua `fields` table (fields.MX64_voltage.value etc.) and hyphenated names are not
    # exposed there (they resolve for `IS` but come back nil in the template).
    field  SFFloat     MX64_voltage         14.8
    field  SFFloat     MX64_current         4.03             # stall current: datasheet torque/speed => I = torque * vel / voltage
    field  SFFloat     MX64_K               1.8115           # = voltage / datasheet no-load speed (8.17 rad/s)
    field  SFFloat     MX64_damping         0.65
    field  SFFloat     MX64_friction        1.73
    # MX106: hip yaw only (2 total)
    #   K and current (= V/R) from the MX106 group electrical fit (fit quality: moderate,
    #   elec R^2 ~0.67). => maxTorque ~21.8 N.m, maxVelocity ~4.87 rad/s.
    field  SFFloat     MX106_voltage        14.8
    field  SFFloat     MX106_current        7.18             # = voltage / group R (2.0625 ohm)
    field  SFFloat     MX106_K              3.0404           # group back-EMF constant
    field  SFFloat     MX106_damping        1.23             # bench value, unchanged
    field  SFFloat     MX106_friction       2.55             # bench value, unchanged
    # XH540: hip roll/pitch, knee, ankle pitch/roll (10 total)
    #   K and current (= V/R) from the XH540 group electrical fit (good fit, elec R^2 ~0.97).
    #   => maxTorque ~20.3 N.m, maxVelocity ~5.40 rad/s. The group K_simultaneous is a
    #   load-biased diagnostic (see servo_system_id.py) and is deliberately not used.
    #   damping/friction have no bench measurement for XH540 yet; MX106 values used as a stand-in.
    field  SFFloat     XH540_voltage        14.8
    field  SFFloat     XH540_current        7.40             # = voltage / group R (1.9991 ohm)
    field  SFFloat     XH540_K              2.7391           # group back-EMF constant
    field  SFFloat     XH540_damping        1.23             # MX106 bench value used as stand-in, pending XH540 bench data
    field  SFFloat     XH540_friction       2.55             # MX106 bench value used as stand-in, pending XH540 bench data
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
  ]''')

# Add recognition colors
filedata = filedata.replace('selfCollision IS selfCollision', '''selfCollision IS selfCollision
    recognitionColors IS recognitionColors''')

# Replace all servo parameters with the correct motor's constants and change to HingeJointWithBacklash
# MX64 used in shoulder roll, shoulder pitch, elbow, neck yaw and head pitch (8 total)
# MX106 used in hip yaw (2 total)
# XH540 used in hip roll, hip pitch, knee, ankle pitch and ankle roll (10 total)

MOTOR_TYPE = {
    'left_shoulder_pitch': 'MX64', 'right_shoulder_pitch': 'MX64',
    'left_shoulder_roll': 'MX64', 'right_shoulder_roll': 'MX64',
    'left_elbow_pitch': 'MX64', 'right_elbow_pitch': 'MX64',
    'neck_yaw': 'MX64', 'head_pitch': 'MX64',
    'left_hip_yaw': 'MX106', 'right_hip_yaw': 'MX106',
    'left_hip_roll': 'XH540', 'right_hip_roll': 'XH540',
    'left_hip_pitch': 'XH540', 'right_hip_pitch': 'XH540',
    'left_knee_pitch': 'XH540', 'right_knee_pitch': 'XH540',
    'left_ankle_pitch': 'XH540', 'right_ankle_pitch': 'XH540',
    'left_ankle_roll': 'XH540', 'right_ankle_roll': 'XH540',
}

# Fixed nominal limits used when useVoltageModel is FALSE. These equal what the
# electrical model computes (maxTorque = K*current, maxVelocity = voltage/K), so the
# two toggle states agree. maxVelocity here must match Webots.yaml's max_velocity_*.
NOMINAL = {
    'MX64':  {'maxVelocity': 8.17,  'maxTorque': 7.30},
    'MX106': {'maxVelocity': 4.87,  'maxTorque': 21.83},
    'XH540': {'maxVelocity': 5.40,  'maxTorque': 20.27},
}

# One HingeJoint's motor block: from its jointParameters up to its own maxTorque.
# Non-greedy so it stops at this joint's own maxTorque, not a nested child joint's.
JOINT_BLOCK_RE = re.compile(r'jointParameters HingeJointParameters \{.*?maxTorque 1\.0', re.DOTALL)

def inject_motor_params(match):
    block = match.group(0)
    joint_name = re.search(r'name "([^"]+)"', block).group(1)
    motor = MOTOR_TYPE[joint_name]
    block = block.replace(
        'HingeJointParameters {',
        f'HingeJointParameters {{\n'
        f'                    dampingConstant IS {motor}_damping\n'
        f'                    staticFriction IS {motor}_friction')
    # maxTorque/maxVelocity: fixed nominal limits unless useVoltageModel is TRUE, in which
    # case they are derived from the electrical fields (stall torque = K*I, no-load speed =
    # V/K) so voltage/current drive the model. Evaluated by Webots' proto templater
    # (%{= ... }%) at instantiation. The `cond and A or B` idiom is safe here because A is
    # always a positive number (never nil/false/0).
    # NB: must be dot access (fields.NAME.value). Webots scans the template for field
    # references to decide which fields to expose; it detects `fields.NAME` but not the
    # bracket form `fields["NAME"]`, which comes back nil. Dot access requires the
    # identifier-safe underscore field names above (a hyphen would parse as subtraction).
    nominal = NOMINAL[motor]
    block = block.replace(
        'maxVelocity 20.0',
        f'maxVelocity %{{= fields.useVoltageModel.value '
        f'and (fields.{motor}_voltage.value / fields.{motor}_K.value) '
        f'or {nominal["maxVelocity"]} }}%')
    block = block.replace(
        'maxTorque 1.0',
        f'maxTorque %{{= fields.useVoltageModel.value '
        f'and (fields.{motor}_K.value * fields.{motor}_current.value) '
        f'or {nominal["maxTorque"]} }}%')
    return block

filedata = JOINT_BLOCK_RE.sub(inject_motor_params, filedata)
filedata = filedata.replace(
    "PositionSensor {", "PositionSensor {\n                 resolution IS DYNAMIXEL-RESOLUTION")

# Add gyro and accelerometer to torso
filedata = filedata.replace('''recognitionColors IS recognitionColors
    children [''', '''recognitionColors IS recognitionColors
    children [
                            %{
                              if fields.name.value ~= '' then
                                -- name is supposed to be something like "red player 2" or "blue player 1"
                                local words = {}
                                for word in fields.name.value:gmatch("%w+") do table.insert(words, word) end
                                local color = words[1]
                                local number = words[3]
                            }%
                            Transform {
                              translation 0.00250000 0.000000 -0.0010000
                              rotation 0.000000 0.000000 1.000000 1.570790
                              children [
                                JerseyFront {
                                  jerseyFrontTexture [
                                  %{='"./NUgus_textures/NUgus_front_' .. tostring(color)  .. '_' .. tostring(number) .. '.jpg"'}%
                                  ]
                                }
                              ]
                            }

                            Transform {
                              translation -0.00250000 0.000000 -0.0010000
                              rotation 0.000000 0.000000 1.000000 1.570790
                              children [
                                JerseyBack {
                                  jerseyBackTexture [
                                  %{='"./NUgus_textures/NUgus_back_' .. tostring(color)  .. '_' .. tostring(number) .. '.jpg"'}%
                                  %{ end }%
                                  ]
                                }
                              ]
                            }
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
                        # Set the recognition parameters from the left camera
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
                        # Allows the robot to detect objects in the world
                        recognition DEF recognition Recognition {
                          frameThickness 0       # Remove bounding boxes
                          segmentation TRUE      # Add segmentation
                        }
                      }
                    ]
                    name "left_camera"''')

# Remove camera frames (causing physics issues in webots)
filedata = filedata.replace('''name "left_camera"
                    boundingObject Pose {
                      translation 0.000000 0.000000 0.000005
                      children [
                        Box {
                           size 0.000010 0.000010 0.000010
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.000000
                      centerOfMass [ -0.000000 0.000000 0.000005 ]
                    }''', '')

filedata = filedata.replace('''name "right_camera"
                    boundingObject Pose {
                      translation 0.000000 0.000000 0.000005
                      children [
                        Box {
                           size 0.000010 0.000010 0.000010
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.000000
                      centerOfMass [ -0.000000 0.000000 0.000005 ]
                    }''', '')

# Add touch sensors to feet
filedata = filedata.replace(''']
                                        name "right_foot"''',  '''
                                          # Define four touch sensors for the right foot
                                          # Back right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0767345 -0.0867505
                                            name "right_touch_sensor_br"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            # Sensor Type
                                            # Bumper returns a 1 when collision is detected
                                            type "bumper"
                                          }
                                          
                                          # Back left touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 0.0548465  -0.0867505
                                            name "right_touch_sensor_bl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front left touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 0.0548465 0.129071
                                            name "right_touch_sensor_fl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995  -0.0767345 0.129071
                                            name "right_touch_sensor_fr"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                        ]
                                        name "right_foot [foot]"''')

filedata = filedata.replace(''']
                                        name "left_foot"''',  '''
                                          # Define four touch sensors for the left foot
                                          # Back right touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0548465 -0.0867505
                                            name "left_touch_sensor_br"
                                            children [
                                              # Define shape for all touch sensors
                                              DEF touch_shape Shape{
                                                appearance PBRAppearance {
                                                  baseColor 0.125 0.125 0.125
                                                  transparency 0.000000
                                                  roughness 1.000000
                                                  metalness 0
                                                  emissiveColor 0.000000 0.000000 0.000000
                                                }
                                              }
                                            ]
                                            # Bounding object uses geometry from touch_shape
                                            boundingObject DEF touch_box Transform {
                                              translation 0.0 0.0 0.0
                                              rotation 0.0 0.0 1.0 1.571
                                              children [
                                                Capsule {
                                                  bottom      TRUE   # {TRUE, FALSE}
                                                  height      0.001  # [0, inf)
                                                  radius      0.009  # [0, inf)
                                                  side        TRUE   # {TRUE, FALSE}
                                                  top         FALSE  # {TRUE, FALSE}
                                                  subdivision 12     # [3, inf)
                                                }
                                              ]
                                            }
                                            # Define physics for all touch sensors
                                            physics DEF touch_physics Physics{
                                              density -1
                                              mass 0.02
                                              centerOfMass 0.0 0.0 0.0
                                            }
                                            type "bumper"
                                          }
                                          # Back left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 0.0767345 -0.0867505
                                            name "left_touch_sensor_bl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 0.0767345 0.129071
                                            name "left_touch_sensor_fl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front right touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0548465 0.129071
                                            name "left_touch_sensor_fr"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                        ]
                                        name "left_foot [foot]"''')
# Rename limbs
filedata = filedata.replace(
    '''name "right_shoulder_pitch"''', '''name "right_shoulder_pitch [shoulder]"''')
filedata = filedata.replace(
    '''name "left_shoulder_pitch"''', '''name "left_shoulder_pitch [shoulder]"''')
filedata = filedata.replace(
    '''name "right_hip_roll"''', '''name "right_hip_roll [hip]"''')
filedata = filedata.replace(
    '''name "left_hip_roll"''', '''name "left_hip_roll [hip]"''')

# Fix naming issue of bounding object caused by urdf2webots tool
filedata = filedata.replace("boundingObject Pose", "boundingObject Transform")

# Fix issue with mass of bounding object
filedata = filedata.replace("mass 0.000000", "mass 1e-8")
filedata = filedata.replace("mass -1", "mass 1e-8")

# Update colours
filedata = filedata.replace(
    "baseColor 0.286275 0.286275 0.286275", "baseColor 0.125 0.125 0.125")

# Write the update proto file
with open(proto_file_path, 'w') as file:
    file.write(filedata)

print("Proto file updated")
