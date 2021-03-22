# Webots

This is the Webots world, models and controllers for simulating the NUbots robots.

The field environment was developed by Cyberbotics for RoboCup 2021 (Humanoid League). 

## File System

| Folder      | Description                                                                                                        |
| ----------- | ------------------------------------------------------------------------------------------------------------------ |
| controllers | Controller files written in C++. These connect to a model, eg the robot model, and determine how it works.         |
| protos      | Proto files, which define the models for objects.                                                                  |
| worlds      | World files, which are opened in Webots and define what is in the environment and how the environment initialises. |

## Set Up

For Ubuntu (tested on Linux Mint).

To get the webots simulator, pull the docker image

```sh
docker pull cyberbotics/webots
```

Get the NUbots world files

```sh
git clone https://github.com/NUbots/Webots
```

Navigate to the NUbots Webots folder

```sh
cd Webots
```

Run the docker container with GUI and mount the NUbots Webots folder so it can be opened inside Docker.

```sh
docker run -it -e DISPLAY -v "$(pwd):/root/" -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest webots
```

Note: webots may give you an error if there is an issue with the graphics card. Make sure you have the latest drivers. Intel graphics cards are not recommended.

Webots should open. Click on `File->Open World...`. 

In the file explorer pop up, click on `root`. Navigate into the `worlds` folder and open `kid.wbt`.
