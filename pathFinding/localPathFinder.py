"""
Runs a local path finder instance (no ROS).  This is convenient for debugging.
"""
import gui
from gui.pathFinder.localPathFinderInterface import LocalPathFinderInterface
from gui.pathFinder.pathFindViewer import PathFindViewer
if __name__ == '__main__':
    PathFindViewer(LocalPathFinderInterface(), 800, 800)
    
    gui.startGUI()
