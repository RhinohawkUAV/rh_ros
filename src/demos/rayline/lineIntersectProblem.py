import Tkinter as tk

import numpy as np

import geometry.lineSegment
from geometry.lineSegment import LineSeg
from gui import Drawable


class LineIntersectProblem(Drawable):
    def __init__(self, p1, p2, startPoint, endPoint):
        self._startPoint = np.array(startPoint, np.double)
        self._endPoint = np.array(endPoint, np.double)
        self._line = LineSeg(p1, p2)

    def setStartPoint(self, startPoint):
        self._startPoint = np.array(startPoint)

    def setEndPoint(self, endPoint):
        self._endPoint = endPoint

    def draw(self, canvas, **kwargs):
        intersected = self._line.checkLineIntersection(self._startPoint, self._endPoint)
        if intersected:
            fill = "red"
        else:
            fill = "black"

        self._line.draw(canvas, fill=fill)
        geometry.lineSegment.drawLine(canvas, self._startPoint, self._endPoint, fill="blue",
                                      arrow=tk.LAST)
