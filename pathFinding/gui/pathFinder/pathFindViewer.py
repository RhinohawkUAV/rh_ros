import copy
import os
from random import setstate
import tkFileDialog

from engine import interface
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY
from gui.pathFinder.pathFinderListener import PathFinderListener
from gui.pathFinder.pathfindDrawable import PathFindDrawable
from gui.visualizer import Visualizer


class PathFindViewer(Visualizer, PathFinderListener):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, pathFinderInterface, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self.pointOfInterest = None
        self._pathFinderInterface = pathFinderInterface
        self._pathFinderInterface.setListener(self)
        self._pathFindDrawable = None
        
        self.bindWithTransform('<Key>', self.onKeyPressed)
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Button-1>', self.onLeftClick)
        
#         self._resetDynamicPathFinder = dynamicPathFinder
#         self._dynamicPathFinder = copy.deepcopy(self._resetDynamicPathFinder)
#         self.pointOfInterest = None
#         self.bindWithTransform('<Button-1>', self.onLeftClick)
#         self.bindWithTransform('<Button-3>', self.onRightClick)

    def onKeyPressed(self, point, event):
        key = event.keysym
        if key == "l":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.askopenfilename(defaultextension=".json", initialdir=initialPath)
            if not fileName == '':
                pathFindInput = interface.loadInput(fileName)
                self.setState(pathFindInput[SCENARIO_KEY], pathFindInput[VEHICLE_KEY])

    def setState(self, scenario, vehicle):
        # Show a slight extra buffer around the border
        bounds = scenario.calcBounds()
        centerX = (bounds[0] + bounds[2]) / 2.0
        centerY = (bounds[1] + bounds[3]) / 2.0
        rangeX = bounds[2] - bounds[0]
        rangeY = bounds[3] - bounds[1]        
        self.setView(centerX, centerY, rangeX + 1, rangeY + 1)
        self._pathFinderInterface.initiate(scenario, vehicle)
        self._pathFindDrawable = PathFindDrawable(scenario)
        self.updateDisplay()

    def debug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pathFindDrawable.updateDebug(pastPathSegments, futurePathSegments, filteredPathSegments)
        self.updateDisplay()

    def solution(self, solutionPathSegments, finished):
        self._pathFindDrawable.updateSolution(solutionPathSegments, finished)
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.step()

    def onMouseMotion(self, point, event):
        self.pointOfInterest = point
        self.updateDisplay()

    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self.pointOfInterest, snapDistance=5.0)
