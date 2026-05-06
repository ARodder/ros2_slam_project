# JetRacer ROS 2 conversion

## Introduction

## Contents
This repository contains 4 ROS 2 workspaces with different purposes.

### jetracer_ws
The jetracer_ws contains the code for the custom ROS 2 nodes required for 
making the jetRacer work. These are the custom nodes required to run on the jetracer.
It containes the following nodes:
 - Jetracer_node(car_serial_interp)
 - custom_ekf_node
It also contains a camera node which is not yet operational, and a launch file.

### server_ws
The server_ws contains the code for the nodes that should run offboard, meaning they
should not be running on the jetracer itself, but a secondary computer connected to the
JetRacer over WiFi. This workspace is essentially only for configuring the implementations
of slam_toolbox and nav2. It does not contain any custom made nodes, but configured
launch scripts.

### third_party_ws
The third_party_ws is only a wrapper for any third_party nodes used in the project. 
It contains a reference to the [slam_toolbox github repository](https://github.com/SteveMacenski/slam_toolbox)
and the [slamtec sllidar github repository](https://github.com/Slamtec/sllidar_ros2)

### util_ws
The util_ws contain nodes that are nice to have, but not required for the
standard operation of the JetRacer. It contains the translated odom_calibration node, 
only used for the initial calibration of the JetRacer. The teleop_joy node which is 
only required when remotely operating the JetRacer using a joystick controller. And the
teleop_keyboard node which is only required when remotely operating the JetRacer using
a keyboard.

## Running main application

Assuming the platform is properly setup, with Ubuntu 24.04 and ROS 2 Jazzy installed.
Running the nodes is done as follows:

1. Clone the repository. On the JetRacer itself this should be done using sparse clone:
```bash
git clone --recurse-submodules --sparse https://github.com/ARodder/ros2_slam_project.git
cd ros2_slam_project
git submodule update --init -- third_party_ws/sllidar_ros2
git sparse-checkout set jetracer_ws third_party_ws
```
On the computer running the server, this should be done using sparse clone:
```bash
git clone --sparse https://github.com/ARodder/ros2_slam_project.git
cd ros2_slam_project
git submodule update --init -- third_party_ws/slam_toolbox
git sparse-checkout set server_ws third_party_ws
```
2. Then run the following build command in the ros2_slam_project folder.
```Bash
colcon build
```
3. To use the nodes you need to source the install script. This can be done by running the following command in the terminal:
```Bash
source install/setup.bash
```
TODO: add example command of how to run the specific nodes.

Make sure that the Lidar module is running before starting the nodes. If it is not running or the incorrect
usb port is used, the node will instantly crash.