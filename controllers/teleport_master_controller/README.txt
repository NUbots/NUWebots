To use this controller:

First add the following lines to the world file

DEF control Robot {
  controller "teleport_master_controller"
  controllerArgs [
    "soccerField"
  ]
  supervisor TRUE
}

Next assign the DEF of the RobocupSoccerField object to "soccerField"

Next, assign a unique DEF name for each of the robots that you want to control.
    Also set the supervisor field to TRUE.

You should also add controllerArgs arguments for the control robot following
    argument 0 such that they match the DEF names of the robots that you want
    to control with this particular master controller.
