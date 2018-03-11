import Tkinter as tk

import numpy as np

import geometry.lineSegment
from geometry.lineSegment import LineSeg
from gui import Drawable


class RayLineProblem(Drawable):
    def __init__(self, p1, p2, startPoint, rayDir):
        self._startPoint = np.array(startPoint, np.double)
        self._rayDir = np.array(rayDir, np.double)
        self._line = LineSeg(p1, p2)

    def setStartPoint(self, startPoint):
        self._startPoint = np.array(startPoint)

    def setEndPoint(self, endPoint):
        self._rayDir = endPoint - self._startPoint
        # TODO: normalization should not be necessary
        self._rayDir /= np.linalg.norm(self._rayDir)

    def draw(self, canvas, **kwargs):
        intersected = self._line.checkIntersection(self._startPoint, self._rayDir)
        if intersected:
            fill = "red"
        else:
            fill = "black"

        self._line.draw(canvas, fill=fill)
        geometry.lineSegment.drawLine(canvas, self._startPoint, self._startPoint + self._rayDir * 20.0, fill="blue",
                                      arrow=tk.LAST)
