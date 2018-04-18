import tkFileDialog

from boundaryBuilder import BoundaryBuilder
from engine.geometry.pathSegment.lineSegmentObstacleData import LineSegmentObstacleData
from engine.interface import utils
from nfzBuilder import NFZBuilder
from nfzPointMover import NFZPointMover
from noFlyMover import NoFlyMover
from pathSegmentTester import PathSegmentTester
from pointToPointEdit import PointToPointEdit
from pointToPointEditor import PointToPointEditor
from .initialPathFindingEdit import InitialPathFindingEdit
from ..core import Drawable
from ..visualizer import Visualizer


class ScenarioEditor(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._initialPathFindingEdit = InitialPathFindingEdit()
        self._pointToPointEdit = PointToPointEdit()
        self._nfzBuilder = NFZBuilder(self._initialPathFindingEdit)
        self._nfzMover = NoFlyMover(self._initialPathFindingEdit)
        self._nfzPointMover = NFZPointMover(self._initialPathFindingEdit)
        self._boundaryBuilder = BoundaryBuilder(self._initialPathFindingEdit)
        self._pointToPointEditor = PointToPointEditor(self._pointToPointEdit)
        self._pathSegmentTester = PathSegmentTester(self._initialPathFindingEdit, LineSegmentObstacleData())

        self._modeMap = {"i": self._nfzBuilder,
                         "m": self._nfzMover,
                         "p": self._nfzPointMover,
                         "b": self._boundaryBuilder,
                         "w": self._pointToPointEditor,
                         "t": self._pathSegmentTester
                         }
        self._mode = self._nfzBuilder

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
            self._mode = self._modeMap[key]
            self._mode.onSwitch()
            self._mode.onMotion(point)
        elif key == "s":
            fileName = tkFileDialog.asksaveasfilename(defaultextension="json", initialdir="scenarios")
            if not fileName == '':
                utils.saveScenario(fileName, self._initialPathFindingEdit, self._pointToPointEdit)
        elif key == "l":
            fileName = tkFileDialog.askopenfilename(defaultextension="json", initialdir="scenarios")
            if not fileName == '':
                (initialInput, pointToPointInput) = utils.loadScenario(fileName)
                self._initialPathFindingEdit.setToInput(initialInput)
                self._pointToPointEdit.setToInput(pointToPointInput)
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
        self._initialPathFindingEdit.draw(canvas, **kwargs)
        self._pointToPointEdit.draw(canvas, color="blue")
        self._mode.draw(canvas, **kwargs)
