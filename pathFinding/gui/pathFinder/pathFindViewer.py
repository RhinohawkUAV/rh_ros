from numpy.random.mtrand import np
import os
import tkFileDialog

from constants import COURSE_DIM
from engine import interface
import engine
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY
from engine.interface.generator import Generator
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.vehicle import DEFAULT_VEHICLE
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
        self._lastScenario = None
        self._showFiltered = False
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
        self.bindWithTransform('<Button-3>', self.onRightClick)

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
        elif key == "z":
            self._showFiltered = not self._showFiltered
            self.updateDisplay()

    def setStateRandom(self):
        boundaryPoints = np.array([(-COURSE_DIM / 2.0, -COURSE_DIM / 2.0),
                                  (-COURSE_DIM / 2.0, COURSE_DIM / 2.0),
                                  (COURSE_DIM / 2.0, COURSE_DIM / 2.0),
                                  (COURSE_DIM / 2.0, -COURSE_DIM / 2.0)], np.double)
        startPoint = boundaryPoints[0] * 0.8
        waypoints = []
        waypoints.append(boundaryPoints[2] * 0.8)
        waypoints.append(boundaryPoints[1] * 0.8)
        waypoints.append(boundaryPoints[3] * 0.8)
        startVelocity = calcs.unit(waypoints[0] - startPoint) * self._vehicle.maxSpeed
        
        generator = Generator(boundaryPoints,
                              startPoint,
                              waypoints,
                              (self._vehicle.maxSpeed,
                               self._vehicle.maxSpeed * 0.9,
                               self._vehicle.maxSpeed * 0.8))
        generator.setGenerationInfo(COURSE_DIM / 10.0,
                                    self._vehicle.maxSpeed,
                                    0.0,
                                    COURSE_DIM / 20.0)
                                    
        generator.generate(20)
        generator.setGenerationInfo(COURSE_DIM / 15.0,
                                    np.array((0.0, 0.0), np.double),
                                    0.15,
                                    COURSE_DIM / 15.0)
        generator.generate(10)
         
        roads = []
        self.setScenario(Scenario(boundaryPoints, generator.polyNFZs, generator.circularNFZs, roads, startPoint, startVelocity, waypoints))
         
    def setScenario(self, scenario):
        self._lastScenario = scenario
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
        self._pathFindDrawable = PathFindDrawable(self._params, self._vehicle, scenario)
        self.updateDisplay()

    def debug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pathFindDrawable.updateDebug(pastPathSegments, futurePathSegments, filteredPathSegments)
        self.updateDisplay()

    def solution(self, solutionsWaypoints, solutionPathSegments, finished):
        self._pathFindDrawable.updateDebug([], [], [])
        self._pathFindDrawable.updateSolution(solutionsWaypoints, solutionPathSegments, finished)
        print "Solution Time: " + str(solutionPathSegments[-1].startTime + solutionPathSegments[-1].elapsedTime)
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.stepProblem()
        
    def onRightClick(self, point, event):
        if self._lastScenario is not None:
            self.setScenario(self._lastScenario)
        
    def onMouseMotion(self, point, event):
        self._pointOfInterest = point
        self.updateDisplay()

    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self._pointOfInterest, snapDistance=120.0, showFiltered=self._showFiltered)
