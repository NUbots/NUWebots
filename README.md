# Webots

This is the Webots world, models and controllers for simulating the NUbots robots.

The field environment was developed by Cyberbotics for RoboCup 2021 (Humanoid League). 

## Set Up

On Windows, use Ubuntu in WSl. You can follow the instructions for setting it up on the [Getting Started NUbook page](https://nubook.nubots.net/guides/main/getting-started)

1. Open a terminal. Clone the [NUbots/Webots](https://github.com/NUbots/Webots/) repository by running

    ```sh
    git clone https://github.com/NUbots/Webots
    ```

2. Compile the controllers with cmake
    
    1. Make a build folder and move into it
    
    ```sh
    mkdir build && cd build
    ```
    
    2. Configure and generate the build files

    ```sh
    cmake ..
    ```

    3. Compile the code
    
    ```sh
    make
    ```

3. Download Webots from source using the instructions on [GitHub](https://github.com/cyberbotics/webots/wiki#installation-of-the-webots-development-environment). 

4. Launch Webots and click on `File->Open World...`. 

    In the pop-up, navigate to the [NUbots/Webots](https://github.com/NUbots/Webots/) repository on your computer and open the `worlds/kid.wbt` world file.

## File System

| Folder      | Description                                                                                                        |
| ----------- | ------------------------------------------------------------------------------------------------------------------ |
| controllers | Controller files written in C++. These connect to a model, eg the robot model, and determine how it works.         |
| protos      | Proto files, which define the models for objects.                                                                  |
| worlds      | World files, which are opened in Webots and define what is in the environment and how the environment initialises. |
