# Webots

This is the Webots world, models and controllers for simulating the NUbots robots.
It also contains a fork of the [TC Webots fork](https://github.com/RoboCup-Humanoid-TC/webots), with some changes. This allows us to run the Official RoboCup Webots simulation environment with our robot.

The field environment was developed by Cyberbotics and the Humanoid TC for RoboCup 2021 (Humanoid League).

## File System

| Folder         | Description                                                                                                                          |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| controllers    | Controller files written in C++. These connect to a model, eg the robot model, and determine how it works.                           |
| protos         | Proto files, which define the models for objects.                                                                                    |
| worlds         | World files, which are opened in Webots and define what is in the environment and how the environment initialises.                   |
| shared/utility | C++ utility functions.                                                                                                               |
| scripts        | Scripts to make certain tasks easier (like creating new controllers).                                                                |
| webots         | The Webots simulator and RoboCup environment, from [the TC Webots fork](https://github.com/RoboCup-Humanoid-TC/webots) with changes. |

## Set Up

For set up information, visit the [Webots NUbook page](https://nubook.nubots.net/guides/tools/webots-setup).