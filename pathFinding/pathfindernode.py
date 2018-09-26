#!/usr/bin/env python
import rospy
from ros.rosPathFinderManager import RosPathFinderManager

if __name__ == '__main__':
    pathFinder = RosPathFinderManager()
    rospy.spin()
