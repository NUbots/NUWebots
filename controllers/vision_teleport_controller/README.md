# Vision Teleport Controller

This controller is used in the vision collection world, `vision_collection.wbt`. It collects vision data with segmented images, and is designed to be used with the NUgus robot. it gets raw images, segmentation masks and lens parameters for each camera of the NUgus robot that the controller is attached to. The robots and ball are teleported randomly around the field, with checks for intersections. 

# Usage

The `vision_collection.wbt` world is an example of how to use this controller.

In the world file, add a NUgus robot. This robot will be the robot that collects vision data. Specify the controller as this controller (`vision_teleport_controller`). The controllerArgs need to include the objects used in the controller. The first arg should be the DEF of the RobocupSoccerField (make sure you give it a DEF in your world file!). The second is this robot's DEF. After this arg, add all of the other objects that you want to teleport around the scene (i.e. the ball, other robots) and by naming their DEF individually. Any robots listed, including this robot should have `supervisor` set to `TRUE`.

```vrml
controller "vision_teleport_controller"
controllerArgs [
  <RobocupSoccerField DEF>
  <THIS ROBOT>
  <OTHER ROBOTS/OBJECTS...>
]
supervisor TRUE
```

# Colours

The colours used in the `vision_collection.wbt` are as follows

| Object                    | Colour                  |
| ------------------------- | ----------------------- |
| Ball                      | Red [1 0 0]             |
| Goal posts (large)        | Yellow [1 1 0]          |
| Goal posts (small)        | Dark yellow [0.5 0.5 0] |
| Field                     | Green [0 1 0]           |
| Field lines               | White [1 1 1]           |
| This robot                | Grey [0.5 0.5 0.5]      |
| Other robots (no colours) | Dark grey [0.3 0.3 0.3] |
| Other robots (red team)   | Magenta [1 0 1]         |
| Other robots (blue team)  | Cyan [0 1 1]            |
