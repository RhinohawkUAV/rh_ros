import os
from threading import Condition, Lock
import time
import tkFileDialog

import Tkinter as tk
from constants import COURSE_DIM
from engine import interface
from engine.interface import scenario, generator
from engine.interface.generator import ObstacleGenerator
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.vehicle import DEFAULT_VEHICLE
import gui
from gui.pathFinder.pathfindDrawable import PathFindDrawable
from gui.simulator.simulator import SimManager
from gui.visualizer import Visualizer
from utils import profile


class PathFindViewer(Visualizer):
    """
    Visualizes the dynamic path finding process.
    """

    def __init__(self, pathFinderInterface, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self.title("Path Finding")
        self._showFiltered = False
        self._bestPath = None
        self._params = None
        self._vehicle = None
        self._scenario = None
        self._pointOfInterest = None
        self._pathFinderInterface = pathFinderInterface
        self._pathFinderInterface.addListeners(self.inputAccepted, self.stepPerformed, self.solved)
        self._pathFindDrawable = None
        
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
                (params, scenario, vehicle) = interface.load(fileName)
                self._pathFinderInterface.submitProblem(params, scenario, vehicle)
        elif key == "s":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.asksaveasfilename(defaultextension=".json", initialdir=initialPath)
            
            if isinstance(fileName, basestring) and not fileName == '':
                interface.save(fileName, self._params, self._scenario, self._vehicle)
        elif key == "r":
            scenario = generator.genStandardHardScenario()
            self._pathFinderInterface.submitProblem(DEFAULT_PARAMS, scenario, DEFAULT_VEHICLE)
        elif key == "i":
            self.iterateScenario()
        elif key == "p":
            print profile.result()
            profile.printAggregate()
        elif key == "y":
            self._pathFinderInterface.solveProblem(self._params, self._scenario, self._vehicle, 5.0)
        elif key == "z":
            self._showFiltered = not self._showFiltered
            self.updateDisplay()
    
    def iterateScenario(self):
        if self._bestPath is not None:
            generator = ObstacleGenerator(self._params, self._scenario, self._vehicle)
            generator.setGenerationInfo(COURSE_DIM / 10.0,
                            1.0,
                            0.0)
            generator.block(self._bestPath.pathSegments)
            self._pathFinderInterface.submitProblem(self._params, self._scenario, self._vehicle)
    
    def inputAccepted(self, params, scenario, vehicle):
        self._bestPath = None
        self._params = params
        self._scenario = scenario
        self._vehicle = vehicle
        
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
        self._bestPath = bestPath

        self.updateDisplay()

    def solved(self, bestPath):
        self._pathFindDrawable.update(True, bestPath, [], [], [])
        self._bestPath = bestPath
        self.updateDisplay()

    def onLeftClick(self, point, event):
        self._pathFinderInterface.stepProblem()
    
    def onControlLeftPress(self, point, event):
        self._pathFinderInterface.stepProblem(10)
    
    def onRightClick(self, point, event):
        if self._scenario is not None:
            self._pathFinderInterface.submitProblem(self._params, self._scenario, self._vehicle)
    
    def onMouseMotion(self, point, event):
        self._pointOfInterest = point
        self.updateDisplay()
        
    def updateDisplay(self):
        if self._pathFindDrawable is not None:
            self.drawToCanvas(self._pathFindDrawable, pointOfInterest=self._pointOfInterest, snapDistance=120.0, showFiltered=self._showFiltered)
