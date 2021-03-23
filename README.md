# Webots

This is the Webots world, models and controllers for simulating the NUbots robots.

The field environment was developed by Cyberbotics for RoboCup 2021 (Humanoid League). 

## Set Up

1. Clone the [NUbots/Webots](https://github.com/NUbots/Webots/) repository by running

    ```sh
    git clone https://github.com/NUbots/Webots
    ```

    in the terminal. On Windows you can do this in [Git Bash](https://gitforwindows.org/).

2. Download Webots from the [Cyberbotics website](https://cyberbotics.com/). 

3. Open Webots and click on `File->Open World...`. 

    In the pop-up, navigate to the [NUbots/Webots](https://github.com/NUbots/Webots/) repository on your computer and open the `worlds/kid.wbt` world file.

## File System

| Folder      | Description                                                                                                        |
| ----------- | ------------------------------------------------------------------------------------------------------------------ |
| controllers | Controller files written in C++. These connect to a model, eg the robot model, and determine how it works.         |
| protos      | Proto files, which define the models for objects.                                                                  |
| worlds      | World files, which are opened in Webots and define what is in the environment and how the environment initialises. |
