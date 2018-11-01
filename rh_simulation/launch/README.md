# Launch files


| File | Description
|----------------------
| compainion.launch | Single node compainion computer
| proxy.launch | Single node with mavros as a proxy to qgroundcontrol
| ros.launch | Two nodes, ros pipeline portion
| simulation.launch | Two nodes, gazebo simulator



# Two nodes

run roscore on one node and then point the other node to the master

```
roscore
```

```
export ROS_MASTER_URI=http://W520:11311/
roslaunch ...
```
