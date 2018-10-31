# RhinoHawkUAV Vehicle Web Application

A browser-based application for interacting with vehicle telemetry.
Ideal usage is during competition, and to make development easier.
Front-end uses *roslibjs* to interact with standard and custom ROS nodes.

## Usage
### ROS Server Setup
Launch ROS Bridge. Always do this. 
```
$ roscore
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

### Use ROSlaunch to launch a pipeline
See latest vehicle pipelines for run details, such as repo *obc-2016-ros*.

### Use ROSbag to play a BAG file
Optional, used for post-mission analysis. 
```
$ rosbag play /path/to/bag/file.bag
```

### Client i.e. another PC on the LAN, or localhost
Open *visionTesting.html* in any browser. Firefox seems to handle the data better. 
