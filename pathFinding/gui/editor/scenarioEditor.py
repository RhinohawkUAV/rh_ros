import tkFileDialog

import numpy as np

from boundaryBuilder import BoundaryBuilder
from engine.interface import utils
from gui.editor.pointToPointEditor import PointToPointEditor
from nfzBuilder import NFZBuilder
from nfzPointMover import NFZPointMover
from noFlyMover import NoFlyMover
from pointToPointEdit import PointToPointEdit
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

        self._modeMap = {"i": self._nfzBuilder,
                         "m": self._nfzMover,
                         "p": self._nfzPointMover,
                         "b": self._boundaryBuilder,
                         "w": self._pointToPointEditor
                         }
        self._mode = self._nfzBuilder
        self.bind('<Key>', self.onKeyPressed)
        self.bind('<ButtonPress-1>', self.onLeftPress)
        self.bind('<ButtonRelease-1>', self.onLeftRelease)
        self.bind('<Control-ButtonPress-1>', self.onControlLeftPress)
        self.bind('<Control-ButtonRelease-1>', self.onControlLeftRelease)
        self.bind('<Control-Motion>', self.onControlMouseMotion)

    def onKeyPressed(self, event):
        key = event.keysym
        if key in self._modeMap:
            self._mode = self._modeMap[key]
            point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
            self._mode.onMotion(point)
        elif key == "s":
            fileName = tkFileDialog.asksaveasfilename(defaultextension="json", initialdir="scenarios")
            if not fileName == '':
                utils.saveScenario(fileName, self._initialPathFindingEdit, self._pointToPointEdit)
        else:
            point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
            self._mode.onKey(point, key)
        self.updateDisplay()

    def onLeftPress(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onLeftPress(point)
        self.updateDisplay()

    def onLeftRelease(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onLeftRelease(point)
        self.updateDisplay()

    def onControlLeftPress(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onLeftPress(point, control=True)
        self.updateDisplay()

    def onControlLeftRelease(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onLeftRelease(point, control=True)
        self.updateDisplay()

    def onMouseMotion(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onMotion(point)
        self.updateDisplay()

    def onControlMouseMotion(self, event):
        point = np.array(self.transformCanvasToPoint((event.x, event.y)), np.double)
        self._mode.onMotion(point, control=True)
        self.updateDisplay()

    def updateDisplay(self):
        self.drawToCanvas(self)

    def draw(self, canvas, **kwargs):
        self._initialPathFindingEdit.draw(canvas, **kwargs)
        self._pointToPointEdit.draw(canvas, color="blue")
        self._mode.draw(canvas, **kwargs)
