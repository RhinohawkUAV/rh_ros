import os
import tkFileDialog

from boundaryBuilder import BoundaryBuilder
from constants import COURSE_DIM
from engine import interface
from engine.interface.fileUtils import SCENARIO_KEY, \
    INPUT_PARAMS_KEY, VEHICLE_KEY
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.testScenario import TestScenario
from engine.interface.vehicle import DEFAULT_VEHICLE
from gui.editor.nfzEdit.dnfzBuilder import DNFZBuilder
from gui.editor.pathSegmentTester.pathSegmentTester import PathSegmentTester
from gui.editor.wayPointEditor import WayPointEditor
from nfzEdit import NFZBuilder
from nfzEdit import NFZEditor

from ..core import Drawable
from ..visualizer import Visualizer

TEST_INPUT_KEY = "testScenario"


def save(fileName, params, scenario, vehicle, testScenario):
    interface.saveDict(fileName, {INPUT_PARAMS_KEY:params, SCENARIO_KEY:scenario, VEHICLE_KEY:vehicle, TEST_INPUT_KEY:testScenario})


def loadTest(fileName):
    fileDict = interface.loadDict(fileName)

    if fileDict.has_key(TEST_INPUT_KEY):
        testDict = fileDict[TEST_INPUT_KEY]
        testScenario = TestScenario(testDict["startPoint"],
                                     testDict["startVelocity"],
                                     testDict["targetPoint"],
                                     testDict["velocityOfTarget"])
    else:
        testScenario = TestScenario()
    
    return testScenario


class ScenarioEditor(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._params = DEFAULT_PARAMS
        self._scenario = Scenario()
        self._vehicle = DEFAULT_VEHICLE
        self._testScenario = TestScenario()
        
        self._nfzBuilder = NFZBuilder()
        self._dnfzBuilder = DNFZBuilder()
        self._nfzEditor = NFZEditor()
        self._boundaryBuilder = BoundaryBuilder()
        self._wayPointEditor = WayPointEditor()
        self._pathSegmentTester = PathSegmentTester()

        self._modeMap = {"i": self._nfzBuilder,
                         "d": self._dnfzBuilder,
                         "e": self._nfzEditor,
                         "b": self._boundaryBuilder,
                         "w": self._wayPointEditor,
                         "t": self._pathSegmentTester
                         }
        self._mode = self._nfzBuilder
        self._setModeState()
            
        self.bindWithTransform('<Motion>', self.onMouseMotion)
        self.bindWithTransform('<Key>', self.onKeyPressed)
        self.bindWithTransform('<Control-Key>', self.onCTRLKeyPressed)
        self.bindWithTransform('<ButtonPress-1>', self.onLeftPress)
        self.bindWithTransform('<ButtonRelease-1>', self.onLeftRelease)
        self.bindWithTransform('<Control-ButtonPress-1>', self.onControlLeftPress)
        self.bindWithTransform('<Control-ButtonRelease-1>', self.onControlLeftRelease)
        self.bindWithTransform('<Control-Motion>', self.onControlMouseMotion)

    def onCTRLKeyPressed(self, point, event):
        if event.keysym != "Control_L":
            self.onKeyPressed(point, event, ctrl=True)

    def onKeyPressed(self, point, event, ctrl=False):
        key = event.keysym
        if key in self._modeMap:
            self._mode.onExit()
            self._mode = self._modeMap[key]
            self._setModeState()
            self._mode.onMotion(point)
        elif key == "s":
            # Make sure current mode's state is dumped back to central data structure
            self._mode.onExit()
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))            
            fileName = tkFileDialog.asksaveasfilename(defaultextension=".json", initialdir=initialPath)
            if not fileName == '':
                save(fileName, self._params, self._scenario, self._vehicle, self._testScenario)
        elif key == "l":
            root = os.path.dirname(__file__)
            initialPath = os.path.normpath(os.path.join(root, "../../../scenarios"))                  
            fileName = tkFileDialog.askopenfilename(defaultextension=".json", initialdir=initialPath)
            if not fileName == '':
                (self._params, self._scenario, self._vehicle) = interface.load(fileName)
                self._testScenario = loadTest(fileName)
                # Scale display to loaded scenario dimensions
                bounds = self._scenario.calcBounds()
                if bounds[0] == float("inf"):
                    bounds = 0, 0, COURSE_DIM, COURSE_DIM 
                centerX = (bounds[0] + bounds[2]) / 2.0
                centerY = (bounds[1] + bounds[3]) / 2.0
                rangeX = bounds[2] - bounds[0]
                rangeY = bounds[3] - bounds[1]        
                self.setView(centerX, centerY, rangeX * 1.1, rangeY * 1.1)
                self._setModeState()
                self._mode.onMotion(point)
        else:
            self._mode.onKey(point, key, ctrl=ctrl)
        self.updateDisplay()

    def _setModeState(self):
        self._mode.onSwitch(self._params, self._scenario, self._vehicle, self._testScenario)
        
    def onLeftPress(self, point, event):
        self._mode.onLeftPress(point)
        self.updateDisplay()

    def onLeftRelease(self, point, event):
        self._mode.onLeftRelease(point)
        self.updateDisplay()

    def onControlLeftPress(self, point, event):
        self._mode.onLeftPress(point, control=True)
        self.updateDisplay()

    def onControlLeftRelease(self, point, event):
        self._mode.onLeftRelease(point, control=True)
        self.updateDisplay()

    def onMouseMotion(self, point, event):
        self._mode.onMotion(point)
        self.updateDisplay()

    def onControlMouseMotion(self, point, event):
        self._mode.onMotion(point, control=True)
        self.updateDisplay()

    def updateDisplay(self):
        self.drawToCanvas(self)

    def draw(self, canvas, **kwargs):
        self._mode.draw(canvas, **kwargs)

