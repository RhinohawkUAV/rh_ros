# from sitl_run.sh

sitl_gazebo_dir=~/catkin_ws/src/sitl_gazebo
obc_gazebo=~/catkin_ws/src/obc_gazebo

export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$sitl_gazebo_dir/models:$obc_gazebo/models
export SITL_GAZEBO_PATH=$sitl_gazebo_dir

# The next line would disable online model lookup, can be commented in, in case of unstable behaviou
# export GAZEBO_MODEL_DATABASE_URI=""
