# OBC Software In The Loop simulation

This package is designed to startup a simulation of a quadcopter and camera and attach it to the PX4 autopilot and our ROS image processing pipeline.

PX4 comes with a SITL simulation based on Gazebo.  The plan is to use PX4 sources for the
quadcopter autopilot, ground station, and mavros interfaces.  We'll add a 
camera to a PX4 Iris quad model and use our ROS image pipeline to 
process the camera output and locate Joe in the local reference frame.

To get ready for this make sure you can run the image pipeline from a
webcam.  Try a few Gazebo runs and build your own model.  Run the PX4
SITL examples.

The PX4 developer documentation for SITL and interfacing 
ROS/GAZEBO/PX4/QGroundControl.

https://dev.px4.io/en/simulation/gazebo.html

https://dev.px4.io/en/simulation/ros_interface.html 

## Running 

The simulator and image pipeline are started with roslaunch:

    cd ~/catkin_ws
    source src/obc_gazebo/scripts/setup.sh
    roslaunch obc_gazebo obc.launch

Which should bring up:
* gazebo server and client
* PX4 autopilot and simulation interfaces
* ROS image pipeline

Use `rqt` to view images from ROS topics and preddictions of Joe's location.

Then use `qgroundcontrol` mission planning to make the quad fly.  Or attach 
a joystick to control the quad.

## Installing

Checkout PX4 and get a SILT build working.  

https://dev.px4.io/en/simulation/gazebo.html

Checkout our OBC image processing pipeline and make it work on a laptop and webcam.

https://gitlab.com/NL-outback-challenge-2016/obc-2016-ros/wikis/vision-pipeline-user-guide


