# local build dirs of px4 and obc
firmware=~/src/Firmware
obc_ros=~/catkin_ws

# Launch px4 and gazebo via ros wrappers, pull in px4 models
# https://dev.px4.io/en/simulation/ros_interface.html
source $obc_ros/devel/setup.sh
source $firmware/Tools/setup_gazebo.bash $firmware $firmware/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$firmware/Tools/sitl_gazebo

# Pull in OBC models
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$obc_ros/src/obc_gazebo/models

# Change startup location to Baron Cameron Park Field #9
export PX4_HOME_LAT=38.977710
export PX4_HOME_LON=-77.337480
export PX4_HOME_ALT=400
