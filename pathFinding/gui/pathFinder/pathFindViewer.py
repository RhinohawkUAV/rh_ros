from fileinput import filename
from numpy.random.mtrand import np
import os
import tkFileDialog

from engine import interface
import engine
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.vehicleInput import VehicleInput
from gui.pathFinder.pathFinderListener import PathFinderListener
from gui.pathFinder.pathfindDrawable import PathFindDrawable
from gui.visualizer import Visualizer
from utils import profile


class PathFindViewer(Visualizer, PathFinderListener):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, pathFinderInterface, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._pointOfInterest = None
        self._pathFinderInterface = pathFinderInterface
        self._pathFinderInterface.setListener(self)
        self._pathFindDrawable = None
        self._pathFindInput = None
        self.bindWithTransform('<Key>', self.onKeyPressed)
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Button-1>', self.onLeftClick)

    def onKeyPressed(self, point, event):
        key = event.keysym
        if key == "l":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.askopenfilename(defaultextension=".json", initialdir=initialPath)
            
            if isinstance(fileName, basestring) and not fileName == '':
                self._pathFindInput = interface.loadInput(fileName)
                self.setState(self._pathFindInput[SCENARIO_KEY], self._pathFindInput[VEHICLE_KEY])
        if key == "s":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.asksaveasfilename(defaultextension=".json", initialdir=initialPath)
            
            if isinstance(fileName, basestring) and not fileName == '':
                interface.saveInput(fileName, self._pathFindInput)
        elif key == "r":
            self.setStateRandom()
        elif key == "t":
            print profile.result()
            profile.printAggregate()

    def setStateRandom(self):
        startPoint = (95, 95)
        endPoint = (5, 5)
        maxSpeed = 3.0
        startVelocity = calcs.unit(np.array([-1.0, -1.0], np.double)) * maxSpeed
        startVelocity = (startVelocity[0], startVelocity[1])
        boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]
        
        noFlyZones = engine.utils.genRandomNoFlyZoneInputsHard(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=2.0,
                                                                    startPoint=startPoint, endPoint=endPoint, averageSpeed=maxSpeed)
        
        roads = []
        wayPoints = [endPoint]
        scenario = ScenarioInput(boundaryPoints, noFlyZones, roads, startPoint, startVelocity, wayPoints)
        vehicle = VehicleInput(maxSpeed=maxSpeed, acceleration=1.0)
        self.setState(scenario, vehicle)
        
    def setState(self, scenario, vehicle):
        # Show a slight extra buffer around the border
        bounds = scenario.calcBounds()
        centerX = (bounds[0] + bounds[2]) / 2.0
        centerY = (bounds[1] + bounds[3]) / 2.0
        rangeX = bounds[2] - bounds[0]
        rangeY = bounds[3] - bounds[1]        
        self.setView(centerX, centerY, rangeX + 1, rangeY + 1)
        self._pathFindInput = {}
        self._pathFindInput[SCENARIO_KEY] = scenario
        self._pathFindInput[VEHICLE_KEY] = vehicle
        
        self._pathFinderInterface.submitProblem(scenario, vehicle)
        self._pathFindDrawable = PathFindDrawable(scenario)
        self.updateDisplay()

    def debug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pathFindDrawable.updateDebug(pastPathSegments, futurePathSegments, filteredPathSegments)
        self.updateDisplay()

    def solution(self, solutionPathSegments, finished):
        self._pathFindDrawable.updateSolution(solutionPathSegments, finished)
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.stepProblem()

    def onMouseMotion(self, point, event):
        self._pointOfInterest = point
        self.updateDisplay()

    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self._pointOfInterest, snapDistance=5.0)
