import tkFileDialog

import numpy as np

from boundaryBuilder import BoundaryBuilder
from engine.geometry.pathSegment.arcObstacleData import ArcObstacleData
from engine.interface import utils
from gui.editor import testEdit
from gui.editor.testEdit import TestEdit
from nfzBuilder import NFZBuilder
from nfzPointMover import NFZPointMover
from noFlyMover import NoFlyMover
from pathEdit import PathEdit
from pathEditor import pathEditor
from pathSegmentTester import PathSegmentTester
from .obstacleCourseEdit import ObstacleCourseEdit
from ..core import Drawable
from ..visualizer import Visualizer


class ScenarioEditor(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._obstacleCourseEdit = ObstacleCourseEdit()
        self._pathEdit = PathEdit()
        self._testEdit = TestEdit(np.array((5, 5), np.double), np.array((1, 1), np.double),
                                  np.array((95, 95), np.double),
                                  np.array((0, 0), np.double))
        self._nfzBuilder = NFZBuilder(self._obstacleCourseEdit)
        self._nfzMover = NoFlyMover(self._obstacleCourseEdit)
        self._nfzPointMover = NFZPointMover(self._obstacleCourseEdit)
        self._boundaryBuilder = BoundaryBuilder(self._obstacleCourseEdit)
        self._pathEditor = pathEditor(self._pathEdit)
        self._pathSegmentTester = PathSegmentTester(self._testEdit, self._obstacleCourseEdit, ArcObstacleData())

        self._modeMap = {"i": self._nfzBuilder,
                         "m": self._nfzMover,
                         "p": self._nfzPointMover,
                         "b": self._boundaryBuilder,
                         "w": self._pathEditor,
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
        self._obstacleCourseEdit.draw(canvas, **kwargs)
        self._pathEdit.draw(canvas, color="blue")
        self._mode.draw(canvas, **kwargs)
