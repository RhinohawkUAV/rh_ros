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
    * Upgrade ROS Jade from Gazebo 5 to Gazebo 6 ROS packages.  Use prebuilt debs:
    ```
    sudo apt-get install ros-jade-gazebo6-ros-pkgs
    ```
    * Otherwise see these links
        * http://gazebosim.org/tutorials/?tut=ros_overview
        * http://gazebosim.org/tutorials/?tut=ros_wrapper_versions
        * http://gazebosim.org/tutorials?tut=ros_installing#A.InstallPre-BuiltDebians
   * protobuf compiler
   ```
   sudo apt-get install protobuf-compiler
   ```
   * mavros
   ```
   sudo apt-get install ros-jade-mavros ros-jade-mavros-extras
   ```
   * PX4 http://dev.px4.io
   * qgroundcontrol 

 
* Create catkin workspace and checkout project
    ```
    source /opt/ros/jade/setup.bash
    mkdir ~/catkin_ws
    git clone git@gitlab.com:NL-outback-challenge-2016/obc-2016-ros.git src
    cd ~/catkin_ws/src
    catkin_init_workspace
    git submodule update --init    
    ```
* Build
    ```
    cd ~/catkin_ws
    catkin_make -DCMAKE_MODULE_PATH=/opt/ros/jade/share/cmake_modules/cmake/Modules
    ```
* Run gazebo simulator and ROS nodes.  This should bring up the Gazebo client.  Use the client to make sure the scene is good.  I typically kill the Gazebo client after that as it is a hog.
    ```bash
    cd ~/catkin_ws
    source /usr/share/gazebo/setup.sh
    source devel/setup.sh
    source src/obc_gazebo/setup.sh
    roslaunch obc_gazebo obc.launch
    ```
* Start the PX4 autopilot
    ```
    cd ~/catkin_ws
    ./src/obc_gazebo/scripts/sitl_px4_run.sh
    ```  
* qground control
