"""
Runs a local path finder instance (no ROS).  This is convenient for debugging.
"""
from engine.localPathFinderInterface import LocalPathFinderInterface
import gui
from gui.pathFinder.pathFindViewer import PathFindViewer

PathFindViewer(LocalPathFinderInterface(), 800, 800)

gui.startGUI()
