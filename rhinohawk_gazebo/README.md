# Package for Software In the Loop (SIL) simulations of the Rhinohawk vision pieline.

The simulator and image pipeline are started with roslaunch:

    roslaunch rhinohawk_gazebo rhinohawk.launch

Which should bring up the gazebo client.  Use `rqt` to view images from ROS topics.

This is designed to be used with the [PX4 SIL simulation](http://dev.px4.io/simulation-gazebo.html).  PX4 uses [Gazebo](http://gazebosim.org/) as the physics simulator.  

PX4 uses Gazebo 6.  ROS jade uses Gazebo 5.  A key part of installing is to get ROS working with Gazebo 6.  Here is what I did:

* Install ROS Jade desktop full on Ubunutu 14.04 - http://wiki.ros.org/jade/Installation/Ubuntu
* Upgrade ROS from Gazebo 5 to Gazebo 6 ROS packages.  Install pre-built Debian packages - http://gazebosim.org/tutorials?tut=ros_installing#A.InstallPre-BuiltDebians 
* Do a source build of gazebo_ros_pkgs.  These packages provide the topics that link ROS and Gazebo.  Clone https://github.com/ros-simulation/gazebo_ros_pkgs.git into a catkin workspace and run/build from there.



