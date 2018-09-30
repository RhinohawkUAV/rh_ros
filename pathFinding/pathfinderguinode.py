#!/usr/bin/env python
import rospy

import constants
import gui
from gui.pathFinder.pathFindViewer import PathFindViewer
import pathfinding.msg as pfm
from ros.rosPathFinderInterface import RosPathFinderInterface

if __name__ == '__main__':
    rospy.init_node("pathFinderDebug", anonymous=True)
    pathFinderInterface = RosPathFinderInterface(pfm.GPSCoord(constants.CANBERRA_GPS[0], constants.CANBERRA_GPS[1]))
    PathFindViewer(pathFinderInterface, 800, 800)
    gui.startGUI()