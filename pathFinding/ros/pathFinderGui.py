import rospy
import gui
from gui.pathFinder.pathFindViewer import PathFindViewer
from ros.rosPathFinderInterface import RosPathFinderInterface

    
def main():
    rospy.init_node("pathFinderDebug", anonymous=True)
    PathFindViewer(RosPathFinderInterface(), 800, 800)
    gui.startGUI()
