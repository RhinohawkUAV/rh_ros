from fileinput import filename
from numpy.random.mtrand import np
import os
import tkFileDialog

from constants import COURSE_DIM
from engine import interface
import engine
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.vehicle import Vehicle, DEFAULT_VEHICLE
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
        self._params = DEFAULT_PARAMS
        self._vehicle = DEFAULT_VEHICLE
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
                scenario = self._pathFindInput[SCENARIO_KEY]
                # TODO: Decide how we want to handle saved scenario velocities.  
                # For now we force it to match the vehicle's max velocity
                scenario.startVelocity = calcs.unit(scenario.startVelocity) * self._vehicle.maxSpeed
                self.setScenario(scenario)
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
        startPoint = np.array((-COURSE_DIM / 2.0 * 0.95, -COURSE_DIM / 2.0 * 0.95), np.double)
        endPoint = np.array((COURSE_DIM / 2.0 * 0.95, COURSE_DIM / 2.0 * 0.95), np.double)
        startVelocity = calcs.unit(endPoint - startPoint) * self._vehicle.maxSpeed
        boundaryPoints = [(-COURSE_DIM / 2.0, -COURSE_DIM / 2.0),
                          (-COURSE_DIM / 2.0, COURSE_DIM / 2.0),
                          (COURSE_DIM / 2.0, COURSE_DIM / 2.0), (COURSE_DIM / 2.0, -COURSE_DIM / 2.0)]

        # TODO: Make regular NFZs not dynamic.  Generate dynamic NFZs according to rules.         
        noFlyZones = engine.utils.genRandomNoFlyZoneInputsHard(50,
                                                                    - COURSE_DIM / 2.0 * 0.9,
                                                                    - COURSE_DIM / 2.0 * 0.9,
                                                                    COURSE_DIM * 0.9,
                                                                    COURSE_DIM * 0.9,
                                                                    0.01, 0.1,
                                                                    minSpeed=0.0, maxSpeed=self._vehicle.maxSpeed,
                                                                    startPoint=startPoint, endPoint=endPoint,
                                                                    averageSpeed=self._vehicle.maxSpeed)
         
        dynamicNoFlyZones = []
        roads = []
        wayPoints = [endPoint]
        scenario = Scenario(boundaryPoints, noFlyZones, dynamicNoFlyZones, roads, startPoint, startVelocity, wayPoints)
        self.setScenario(scenario)
         
    def setScenario(self, scenario):
        # Show a slight extra buffer around the border
        bounds = scenario.calcBounds()
        centerX = (bounds[0] + bounds[2]) / 2.0
        centerY = (bounds[1] + bounds[3]) / 2.0
        rangeX = bounds[2] - bounds[0]
        rangeY = bounds[3] - bounds[1]        
        self.setView(centerX, centerY, rangeX * 1.1, rangeY * 1.1)
        self._pathFindInput = {}
        self._pathFindInput[SCENARIO_KEY] = scenario
        self._pathFinderInterface.submitProblem(self._params, scenario, self._vehicle)
        self._pathFindDrawable = PathFindDrawable(scenario)
        self.updateDisplay()

    def debug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pathFindDrawable.updateDebug(pastPathSegments, futurePathSegments, filteredPathSegments)
        self.updateDisplay()

    def solution(self, solutionsWaypoints, solutionPathSegments, finished):
        self._pathFindDrawable.updateSolution(solutionsWaypoints, solutionPathSegments, finished)
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.stepProblem()

    def onMouseMotion(self, point, event):
        self._pointOfInterest = point
        self.updateDisplay()

    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self._pointOfInterest, snapDistance=120.0)
