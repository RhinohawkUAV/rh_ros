# Rhinohawk Autonomous UAV System

This repository contains the ROS code for the Rhinohawk Autonomous UAV System. It should be checked out into the src subdirectory of a Catkin workspace, like so:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone git@github.com:RhinohawkUAV/rh_ros.git
```

## Install

This software has been developed and tested on Ubuntu 14. To get started, create a new [Lubuntu 14](https://lubuntu.net/lubuntu-1404-trusty-tahr-released/) VM, and use the RH ROS [Installer](https://github.com/RhinohawkUAV/rh_ros_installer) to install ROS, the Rhinohawk code, and all the dependencies.

## Build
To build, run `catkin_make` in the Catkin workspace:
```
cd ~/catkin_ws
catkin_make
```

## Run
<!--- (this doesn't work yet, because we don't have a chameleon3 calibration yet)
To run the Rhinohawk System with the Rhinohawk production hardware:
```
CAMERA_NAME=chameleon3 roslaunch rh_autonomy rh.launch
```
--->

### SITL Simulation

Before running SITL, you'll need to set up ArduPilot.

To run the APM SITL, you should start the SITL in terminal 1:
```
cd ~/src/ardupilot/ArduPlane
MAP_SERVICE=MicrosoftSat sim_vehicle.py -v ArduPlane -f quadplane --console --map -L DalbyAU -S 2
```

In terminal 2, start the Rhinohawk System:
```
source ~/catkin_ws/src/rh_ros/rh_simulation/scripts/setup.sh
roslaunch rh_simulation apm.launch
```

In terminal 3, run missions, e.g.:
```
rosrun rh_simulation run_me2018_ez.sh
```
You can also issue individual commands using the Rhinohawk ROS services:
```
rosservice call /rh/command/flyto "{target_lat: 38.977610, target_long: -77.339027, cruise_altitude: 20}"
```

### Ground Control

To use the web-based Rhinohawk Ground Control Station software, first launch the bridge:

```
roslaunch rh_simulation bridge.launch
```
Then open this file in your web browser:
~/catkin_ws/src/rh_ros/rh_ground_control/web/visionTesting.html

### 3DR Solo

To run the Rhinohawk System with a 3DR Solo quadcopter, connect your laptop to the Solo controlller wifi ("Sololink") and then launch using your calibrated camera configuration:
```
CAMERA_NAME=solo4zoom roslaunch rh_autonomy rh.launch
```

## Code Organization

The code is organized into several ROS packages, each containing one or more ROS nodes:

* Core Rhinohawk System
  * **rh_ground_station** - web-based ground station for defining and monitoring missions
  * **rh_msgs** - shared ROS message/service definition, modeled after mavros_msgs package
  * **rh_autonomy** - Rhinohawk autonomous mission execution
    * /rh/state
    * /rh/flight
    * /rh/controller
    * /rh/command
  * **rh_vision** - all machine vision code for camera-based sensing
    * /rh/vision/aruco_detector
    * /rh/vision/image_to_target
  * **rh_camera_driver** - all camera-related code and configurations
    * /camera
    * /camera/image_raw
    * /camera/image_rect
    * /camera/image_mono
* 3dr Solo 
  * **rh_solo** - code and configuration for running the Rhinohawk System on a 3dr Solo quadcopter
  * **gscam** - GStreamer node used for interfacing with the 3DR Solo GoPro camera stream
* Software-In-The-Loop (SITL)
  * **rh_simulation** - code and launch files for running SITL simulations on PX4 and APM

