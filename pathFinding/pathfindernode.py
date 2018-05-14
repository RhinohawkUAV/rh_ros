#!/usr/bin/env python
import rospy

from ros.rosPathFinder import RosPathFinder

if __name__ == '__main__':
    pathFinder = RosPathFinder()
    rospy.spin()
