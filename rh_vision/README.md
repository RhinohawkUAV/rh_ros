# Rhinohawk

ROS nodes for performing computer vision

# Unit tests

To launch unit tests:
```
rostest rh_vision test_aruco.launch
```

In order to debug tests using tools like RQT, the roscore must be launched separately:
```
roscore
rostest --reuse-master --text rh_vision test_aruco.launch
```

