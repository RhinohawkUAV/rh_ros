import Tkinter as tk

import numpy as np

import geometry.lineSegment
from gui import Drawable


class FindNFZIntersectStatic(Drawable):
    def __init__(self, noFlyZone, startPoint, endPoint):
        self._startPoint = np.array(startPoint, np.double)
        self._endPoint = np.array(endPoint, np.double)
        self._speed = 5.0
        self._noFlyZone = noFlyZone

    def setStartPoint(self, startPoint):
        self._startPoint = np.array(startPoint)

    def setEndPoint(self, endPoint):
        self._endPoint = endPoint

    def draw(self, canvas, **kwargs):
        intersected = self._noFlyZone.checkBlocksPath(self._startPoint, self._endPoint, self._speed)
        if intersected:
            fill = "red"
        else:
            fill = "black"

        self._noFlyZone.draw(canvas, fill=fill)
        geometry.lineSegment.drawLine(canvas, self._startPoint, self._endPoint, fill="blue",
                                      arrow=tk.LAST)
