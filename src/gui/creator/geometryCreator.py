import json
import tkFileDialog

import numpy as np

from findPath.geometry import ObstacleCourse
from gui import Drawable
from gui import Visualizer
from nfzBuilder import NFZBuilder
from nfzPointMover import NFZPointMover
from noFlyMover import NoFlyMover


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return obj.__dict__


class GeometryCreator(Visualizer, Drawable):
    """
    The crappiest little editor you've ever seen, but it gets the job done!
    """

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self._obstacleCourse = ObstacleCourse(None, [])
        self._nfzBuilder = NFZBuilder(self._obstacleCourse)
        self._nfzMover = NoFlyMover(self._obstacleCourse)
        self._nfzMove = False
        self._nfzPointMover = NFZPointMover(self, self._obstacleCourse)
        self._modeMap = {"i": self._nfzBuilder, "m": self._nfzMover, "p": self._nfzPointMover}
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
            fileName = tkFileDialog.asksaveasfilename(defaultextension="json", initialdir="../obstacles")
            file = open(fileName, 'w')
            json.dump(self._obstacleCourse, file, cls=NumpyEncoder, indent=4)
            file.close()
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
        self._obstacleCourse.draw(canvas, **kwargs)
        self._mode.draw(canvas, **kwargs)
