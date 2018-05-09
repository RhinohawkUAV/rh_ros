import tkFileDialog

from boundaryBuilder import BoundaryBuilder
from engine import interface
from engine.geometry.pathSegment.arcObstacleData import ArcObstacleData
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.testInput import TestInput
from engine.interface.vehicleInput import VehicleInput
from gui import draw
from gui.editor.pathSegmentTester.pathSegmentTester import PathSegmentTester
from gui.editor.wayPointEditor import WayPointEditor
from nfzEdit import NFZBuilder
from nfzEdit import NFZEditor

from ..core import Drawable
from ..visualizer import Visualizer


class ScenarioEditor(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._inputDict = {
                            interface.SCENARIO_KEY:ScenarioInput(),
                            interface.VEHICLE_KEY:VehicleInput(),
                            interface.TEST_INPUT_KEY:TestInput()
                           }
        self._nfzBuilder = NFZBuilder()
        self._nfzEditor = NFZEditor()
        self._boundaryBuilder = BoundaryBuilder()
        self._wayPointEditor = WayPointEditor()
        self._pathSegmentTester = PathSegmentTester()

        self._modeMap = {"i": self._nfzBuilder,
                         "e": self._nfzEditor,
                         "b": self._boundaryBuilder,
                         "w": self._wayPointEditor,
                         "t": self._pathSegmentTester
                         }
        self._mode = self._nfzBuilder
        self._mode.onSwitch(self._inputDict)
            
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
            self._mode.onSwitch(self._inputDict)
            self._mode.onMotion(point)
        elif key == "s":
            fileName = tkFileDialog.asksaveasfilename(defaultextension=".json", initialdir="scenarios")
            if not fileName == '':
                interface.saveInput(fileName, self._inputDict)
        elif key == "l":
            fileName = tkFileDialog.askopenfilename(defaultextension=".json", initialdir="scenarios")
            if not fileName == '':
                self._inputDict = interface.loadInput(fileName)
            self._mode.onSwitch(self._inputDict)
            self._mode.onMotion(point)
        else:
            self._mode.onKey(point, key, ctrl=ctrl)
        self.updateDisplay()

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
        draw.drawScenario(canvas, self._inputDict[interface.SCENARIO_KEY])
        self._mode.draw(canvas, **kwargs)
