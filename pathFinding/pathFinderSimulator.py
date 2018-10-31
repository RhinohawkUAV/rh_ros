"""
Runs a local path finder instance (no ROS) and a simulator in two separate windows.
"""
import gui
from gui.pathFinder.localPathFinderInterface import LocalPathFinderInterface
from gui.pathFinder.pathFindViewer import PathFindViewer
from gui.simulator.pathFindSimulatorViewer import PathFindSimulatorViewer
if __name__ == '__main__':
    
    localInterface = LocalPathFinderInterface(dumpScenarios=True)
    PathFindViewer(localInterface, 800, 800)
    PathFindSimulatorViewer(localInterface, 800, 800)
    
    gui.startGUI()
