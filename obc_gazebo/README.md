# OBC Software In The Loop simulation

This package is designed to startup a simulation of a quadcopter and camera and attach it to the PX4 autopilot and our ROS image processing pipeline.

## Running 

The simulator and image pipeline are started with roslaunch:

    roslaunch obc_gazebo obc.launch

Which should bring up:
* gazebo server and client
* ROS image pipeline

Use `rqt` to view images from ROS topics.

Then start PX4 and qgroundcontrol to make the quad fly.

## Installing

This is based on the [PX4 SITL](http://dev.px4.io/simulation-gazebo.html) simulation.  PX4 SITL uses [Gazebo](http://gazebosim.org/) as the physics simulator.  

PX4 uses Gazebo 6.  ROS jade uses Gazebo 5.  A key part of installing is to get ROS working with Gazebo 6.

* System dependecies
    * Install ROS Jade desktop full on Ubunutu 14.04 - http://wiki.ros.org/jade/Installation/Ubuntu
    * Upgrade ROS Jade from Gazebo 5 to Gazebo 6 ROS packages.  Use prebuilt debs
        * sudo apt-get install ros-jade-gazebo6-ros-pkgs
    * Otherwise see these links
        * http://gazebosim.org/tutorials/?tut=ros_overview
        * http://gazebosim.org/tutorials/?tut=ros_wrapper_versions
        * http://gazebosim.org/tutorials?tut=ros_installing#A.InstallPre-BuiltDebians 
* Checkout the project
    * clone 
    * update submodules
* Build
    * `cd ~/catkin_ws`
    * `catkin_make -DCMAKE_MODULE_PATH=/opt/ros/jade/share/cmake_modules/cmake/Modules`
* Source env files prior to running
    * `source /usr/share/gazebo/setup.sh`
    * `source ~/catkin_ws/devel/setup.sh`
    * `source ~/catkin_ws/src/obc_gazebo/setup.s`
* Check that the Gazbeo simulation runs without ROS or autopilot.
    * `gazebo --verbose src/obc_gazebo/worlds/obc.world`
* Launch Gazebo and image pipeline from ROS 
    * roslaunch obc_gazebo obc.launch
* PX4
* qground control
 


Obsolete

* Do a source build of gazebo_ros_pkgs.  These packages provide the topics that link ROS and Gazebo.  Clone https://github.com/ros-simulation/gazebo_ros_pkgs.git into a catkin workspace and run/build from there.

