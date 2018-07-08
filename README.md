# ROS packages for Rhinohawk

To get started, run catkin_make in the parent directory.

To start the system for 3dr Solo:
```
CAMERA_NAME=solo4zoom roslaunch rh_autonomy rh.launch
```

## Code Organization

The code is organized into several ROS packages:

* Core Rhinohawk System
  * **camera_driver** - all camera-related code and configurations
  * **rh_msgs** - shared ROS message/service definition, modeled after mavros_msgs package
  * **rh_autonomy** - rhinohawk master controller and centralized state
  * **rh_vision - all machine vision code, currently aruco + image2target. Includes unit tests. 
* 3dr Solo 
  * **obc_solo** - code and configuration for running the Rhinohawk System on a 3dr Solo quadcopter
  * **gscam** - GStreamer node used for interfacting with the 3dr Solo GoPro camera stream
* Software-In-The-Loop (SITL)
  * **obc_gazebo** - code and launch files for running SITL simulations on PX4 and APM
* Legacy & example code, currently not used
  * **obc_pilot**
  * **depth_vision**



