import copy
import os
from threading import Thread
import time
import tkFileDialog

from engine import interface
from engine.interface import generator
from engine.interface.pathFindParams import PathFindParams
from engine.interface.vehicle import DEFAULT_VEHICLE
from gui.simulator.pilot import Pilot
from gui.simulator.simulator import SimManager
from gui.visualizer import Visualizer

SIM_PARAMS = PathFindParams(90.0, 100.0, 200.0, 1.2, 2000.0)


class PathFindSimulatorViewer(Visualizer):

    def __init__(self, pathFinderInterface, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self.title("Simulator")
        self._simulator = SimManager()
        self._pilot = Pilot(pathFinderInterface, self._simulator)
        self._thread = Thread(target=self._render)
        self._thread.start()
        
        self.bindWithTransform('<Key>', self.onKeyPressed)
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Button-1>', self.onLeftClick)
        self.bindWithTransform('<Control-ButtonPress-1>', self.onControlLeftPress)
        self.bindWithTransform('<Button-3>', self.onRightClick)

    def _startScenario(self, params, scenario, vehicle):
        self._resetState = copy.deepcopy((params, scenario, vehicle))
        bounds = scenario.calcBounds()
        centerX = (bounds[0] + bounds[2]) / 2.0
        centerY = (bounds[1] + bounds[3]) / 2.0
        rangeX = bounds[2] - bounds[0]
        rangeY = bounds[3] - bounds[1]        
        self.setView(centerX, centerY, rangeX * 1.1, rangeY * 1.1)
        self._simulator.setState(None, None)
        self._pilot.setState(params, vehicle)
        self._simulator.setState(scenario, vehicle)
        
    def onKeyPressed(self, point, event):
        key = event.keysym
        if key == "l":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.askopenfilename(defaultextension=".json", initialdir=initialPath)
            
            if isinstance(fileName, basestring) and not fileName == '':
                (params, scenario, vehicle) = interface.load(fileName)
                self._startScenario(params, scenario, vehicle)
        elif key == "s":
            # Can always find the scenarios folder relative to this file regardless of how the program is started
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))
            fileName = tkFileDialog.asksaveasfilename(defaultextension=".json", initialdir=initialPath)
            
            if isinstance(fileName, basestring) and not fileName == '':
                interface.save(fileName, self._params, self._scenario, self._vehicle)
        elif key == "r":
            scenario = generator.genStandardHardScenario()
            self._startScenario(SIM_PARAMS, scenario, DEFAULT_VEHICLE)
        elif key == "equal":
            self._simulator.scaleSpeed(1.4)
        elif key == "minus":
            self._simulator.scaleSpeed(1.0 / 1.4)
            
        elif key == "w":
            self._startScenario(*self._resetState)

    def onLeftClick(self, point, event):
        pass
    
    def onControlLeftPress(self, point, event):
        pass
    
    def onRightClick(self, point, event):
        pass
    
    def onMouseMotion(self, point, event):
        pass

    def _render(self):
        while True:
            time.sleep(0.1)
            simState = self._simulator.copyState()
            if simState is not None:
                self.drawInBackground(simState)
