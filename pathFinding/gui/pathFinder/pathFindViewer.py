from numpy.random.mtrand import np
import os
import tkFileDialog

from constants import COURSE_DIM
from engine import interface
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY, \
    INPUT_PARAMS_KEY
from engine.interface.generator import Generator
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.vehicle import DEFAULT_VEHICLE
from gui.pathFinder.pathfindDrawable import PathFindDrawable
from gui.visualizer import Visualizer
from utils import profile


class PathFindViewer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, pathFinderInterface, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._lastScenario = None
        self._showFiltered = False
        self._params = None
        self._vehicle = None
        self._pointOfInterest = None
        self._pathFinderInterface = pathFinderInterface
        self._pathFinderInterface.setListeners(self.inputAccepted, self.stepPerformed)
        self._pathFindDrawable = None
        self._pathFindInput = None
        self.bindWithTransform('<Key>', self.onKeyPressed)
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Button-1>', self.onLeftClick)
        self.bindWithTransform('<Control-ButtonPress-1>', self.onControlLeftPress)
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
                if self._pathFindInput.has_key(VEHICLE_KEY):
                    vehicle = self._pathFindInput[VEHICLE_KEY]
                else:
                    vehicle = DEFAULT_VEHICLE

                if self._pathFindInput.has_key(INPUT_PARAMS_KEY):
                    params = self._pathFindInput[INPUT_PARAMS_KEY]
                else:
                    params = DEFAULT_PARAMS                    

                self._pathFinderInterface.submitProblem(params, scenario, vehicle)
        elif key == "s":
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
        startVelocity = calcs.unit(waypoints[0] - startPoint) * DEFAULT_VEHICLE.maxSpeed
        
        generator = Generator(boundaryPoints,
                              startPoint,
                              waypoints,
                              (DEFAULT_VEHICLE.maxSpeed,
                               DEFAULT_VEHICLE.maxSpeed * 0.9,
                               DEFAULT_VEHICLE.maxSpeed * 0.8))
        generator.setGenerationInfo(COURSE_DIM / 10.0,
                                    DEFAULT_VEHICLE.maxSpeed,
                                    0.0,
                                    COURSE_DIM / 20.0)
                                    
        generator.generate(20)
        generator.setGenerationInfo(COURSE_DIM / 15.0,
                                    np.array((0.0, 0.0), np.double),
                                    0.15,
                                    COURSE_DIM / 15.0)
        generator.generate(10)
         
        roads = []
        
        scenario = Scenario(boundaryPoints, generator.polyNFZs, generator.circularNFZs, roads, startPoint, startVelocity, waypoints)
        self._pathFinderInterface.submitProblem(DEFAULT_PARAMS, scenario, DEFAULT_VEHICLE)
    
    def inputAccepted(self, params, scenario, vehicle):
        self._params = params
        self._scenario = scenario
        self._vehicle = vehicle
        self._lastParams = self._params
        self._lastScenario = self._scenario
        self._lastVehicle = self._vehicle
        # Show a slight extra buffer around the border
        bounds = scenario.calcBounds()
        centerX = (bounds[0] + bounds[2]) / 2.0
        centerY = (bounds[1] + bounds[3]) / 2.0
        rangeX = bounds[2] - bounds[0]
        rangeY = bounds[3] - bounds[1]        
        self.setView(centerX, centerY, rangeX * 1.1, rangeY * 1.1)
        self._pathFindDrawable = PathFindDrawable(params, vehicle, scenario)
        self.updateDisplay()    

    def stepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        self._pathFindDrawable.update(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments)
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.stepProblem()
    
    def onControlLeftPress(self, point, event):
        self._pathFinderInterface.stepProblem(10)
    
    def onRightClick(self, point, event):
        if self._lastScenario is not None:
            self._pathFinderInterface.submitProblem(self._lastParams, self._lastScenario, self._lastVehicle)
        
    def onMouseMotion(self, point, event):
        self._pointOfInterest = point
        self.updateDisplay()

    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self._pointOfInterest, snapDistance=120.0, showFiltered=self._showFiltered)
