import Tkinter as tk

import numpy as np

import gui.draw
from gui import Drawable


class FindNFZIntersectStatic(Drawable):
    def __init__(self, noFlyZone, startPoint, endPoint):
        self._startPoint = np.array(startPoint, np.double)
        self._endPoint = np.array(endPoint, np.double)
        self._speed = 5.0
        self._noFlyZone = noFlyZone

    def setStartPoint(self, startPoint):
        self._startPoint = np.array(startPoint, np.double)

    def setEndPoint(self, endPoint):
        self._endPoint = endPoint

    def draw(self, canvas, **kwargs):
        intersected = self._noFlyZone.checkBlocksPath(self._startPoint, self._endPoint, self._speed)
        if intersected:
            color = "red"
        else:
            color = "black"

        self._noFlyZone.draw(canvas, color=color)
        gui.draw.drawLine(canvas, self._startPoint, self._endPoint, color="blue",
                          arrow=tk.LAST)
