# Rhinohawk System

This repository contains the ROS code for the Rhinohawk System. It should be checked out into the src subdirectory of a catkin workspace, like so:
```
mkdir ~/catkin_ws
cd ~/catkin_ws/
git clone git@gitlab.com:NL-outback-challenge-2016/obc-2016-ros.git src
cd src
catkin_init_workspace
```

## Build
To build, run `catkin_make` in the parent directory:
```
cd ~/catkin_ws
catkin_make
```

## Run
To run the Rhinohawk System with the Rhinohawk production hardware:
```
CAMERA_NAME=chameleon3 roslaunch rh_autonomy rh.launch
```

To run the Rhinohawk System with the 3dr Solo test platform, connect your laptop to the Solo controlller wifi ("Sololink") and then launch using your calibrated camera configuration (calibration documentation is currently available in Google Docs):
```
CAMERA_NAME=solo4zoom roslaunch rh_autonomy rh.launch
```

To run the APM SITL, you should start the SITL in terminal 1:
```
cd ~/src/ardupilot/ArduPlane
MAP_SERVICE=MicrosoftSat sim_vehicle.py -v ArduPlane -f quadplane --console --map -L BaronCameron
```

In terminal 2, start the Rhinohawk System:
```
source ~/catkin_ws/src/obc_gazebo/scripts/setup.sh
roslaunch obc_gazebo apm.launch
```

In terminal 3, issue commands, e.g.:
```
rosservice call /rh/command/flyto "{target_lat: 38.977610, target_long: -77.339027, cruise_altitude: 20}"

```

## Code Organization

The code is organized into several ROS packages:

* Core Rhinohawk System
  * **camera_driver** - all camera-related code and configurations
  * **rh_msgs** - shared ROS message/service definition, modeled after mavros_msgs package
  * **rh_autonomy** - rhinohawk master controller and centralized state
  * **rh_vision** - all machine vision code, currently aruco + image2target. Includes unit tests. 
* 3dr Solo 
  * **obc_solo** - code and configuration for running the Rhinohawk System on a 3dr Solo quadcopter
  * **gscam** - GStreamer node used for interfacting with the 3dr Solo GoPro camera stream
* Software-In-The-Loop (SITL)
  * **obc_gazebo** - code and launch files for running SITL simulations on PX4 and APM
* Legacy & example code, currently not used
  * **obc_pilot**
  * **depth_vision**


