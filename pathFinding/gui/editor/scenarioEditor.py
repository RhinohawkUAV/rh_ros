import tkFileDialog

from boundaryBuilder import BoundaryBuilder
from engine.geometry.pathSegment.arcObstacleData import ArcObstacleData
from engine.interface import utils, debugInput
from gui import draw
from gui.editor import testEdit
from nfzEdit import NFZBuilder
from nfzEdit import NFZEditor
from pathSegmentTester import PathSegmentTester
from gui.editor.wayPointEditor import WayPointEditor

from ..core import Drawable
from ..visualizer import Visualizer


class ScenarioEditor(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._debugInput = debugInput.defaultValue()
        self._nfzBuilder = NFZBuilder()
        self._nfzEditor = NFZEditor()
        self._boundaryBuilder = BoundaryBuilder()
        self._wayPointEditor = WayPointEditor()
        self._pathSegmentTester = PathSegmentTester(ArcObstacleData())

        self._modeMap = {"i": self._nfzBuilder,
                         "e": self._nfzEditor,
                         "b": self._boundaryBuilder,
                         "w": self._wayPointEditor,
                         "t": self._pathSegmentTester
                         }
        self._mode = self._nfzBuilder
        self._mode.onSwitch(self._debugInput)
            
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
            self._mode.onSwitch(self._debugInput)
            self._mode.onMotion(point)
        elif key == "s":
            fileName = tkFileDialog.asksaveasfilename(defaultextension="json", initialdir="scenarios")
            if not fileName == '':
                utils.saveScenario(fileName, self._obstacleCourseEdit, self._pathEdit, {"testInput": self._testEdit})
        elif key == "l":
            fileName = tkFileDialog.askopenfilename(defaultextension="json", initialdir="scenarios")
            if not fileName == '':
                scenario = utils.loadScenario(fileName)
                if scenario[utils.OBSTACLE_INPUT_KEY] is not None:
                    self._obstacleCourseEdit.setToInput(scenario[utils.OBSTACLE_INPUT_KEY])
                if scenario[utils.PATH_INPUT_KEY] is not None:
                    self._pathEdit.setToInput(scenario[utils.PATH_INPUT_KEY])
                if scenario.has_key("testInput") and scenario["testInput"] is not None:
                    self._testEdit.set(testEdit.fromJSONDict(scenario["testInput"]))
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
        draw.drawInput(canvas, self._debugInput)
        self._mode.draw(canvas, **kwargs)
