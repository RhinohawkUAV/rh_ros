#!/usr/bin/env python
import rospy
from ros.rosPathFinderServer import RosPathFinderServer

if __name__ == '__main__':
    server = RosPathFinderServer()
    rospy.spin()
