# ROS packages for Outback Challenge

## Installing

Install [ROS](http://wiki.ros.org/jade/Installation/Ubuntu)

Checkout the project into a ROS workspace.

    sudo apt-get install gphoto2 libgphoto2-dev
    mkdir ~/catkin_ws
    cd ~/catkin_ws
    git clone git@gitlab.com:NL-outback-challenge-2016/obc-2016-ros.git src
    catkin_make


The depth_vision module needs CUDA to build.  You can skip this module in the build (if you don't have cuda installed) like this:

    catkin_make -DCATKIN_BLACKLIST_PACKAGES=depth_vision


## Nodes

* [camera_driver](camera_driver/README.md) - ROS nodes for the image processing pipleine.


