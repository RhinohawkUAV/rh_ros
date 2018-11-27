# Rhinohawk Autonomous UAV System

This repository contains the ROS code for the **Rhinohawk Autonomous UAV System**. [Rhinohawk](http://rhinohawk.com) is a non-profit UAV competition team, based in Reston, VA. 

## Install

This software has been developed and tested on Ubuntu 16. To get started, create a new [Lubuntu 16](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release) VM, and use the RH ROS [Installer](https://github.com/RhinohawkUAV/rh_ros_installer) to install ROS, the Rhinohawk code, and all the other dependencies.

## Build

The software needs to be rebuilt any time the message or service definitions are changed, or if C++ code needs to be recompiled. 

To build all the nodes, run `catkin_make` in the Catkin workspace:
```
cd ~/catkin_ws
catkin_make
```

### Simulation

To run a simulation using SITL you'll need three terminals.

In terminal 1, launch the ground control station:
```
roslaunch rh_ground_control start.launch
```

In terminal 2, start the simulator and the Rhinohawk System:
```
roslaunch rh_simulation apm.launch
```

In terminal 3, you can run pre-defined missions:
```
rosrun rh_simulation run_me2018_ez.sh
```
You can also issue individual commands using the Rhinohawk ROS services:
```
rosservice call /rh/command/flyto "{target_lat: 38.977610, target_long: -77.339027, cruise_altitude: 20}"
```


## Run
To run the Rhinohawk System with the Rhinohawk production hardware, you'll run the ground control station software on a laptop connected via wifi to the UAV.

On the laptop, launch the ground control station:
```
roslaunch rh_ground_control start.launch
```

On the UAV, run the autonomous system:
```
CAMERA_NAME=chameleon3 roslaunch rh_autonomy rh.launch
```


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
  * **camera_driver** - all camera-related code and configurations
    * /camera
    * /camera/image_raw
    * /camera/image_rect
    * /camera/image_mono
* 3dr Solo 
  * **rh_solo** - code and configuration for running the Rhinohawk System on a 3dr Solo quadcopter
* Software-In-The-Loop (SITL)
  * **rh_simulation** - code and launch files for running SITL simulations on PX4 and APM

