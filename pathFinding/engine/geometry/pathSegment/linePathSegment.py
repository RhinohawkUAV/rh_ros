import Tkinter as tk

import numpy as np

from defaultPathSegment import DefaultPathSegment
from engine.geometry import LineSegment
from gui import draw
from pathSegment import PathSegment


class LinePathSegment(DefaultPathSegment):
    def __init__(self, startPoint, speed, time, endPoint, endVelocity):
        PathSegment.__init__(self, time, endPoint, endVelocity)
        self.startPoint = startPoint
        self.speed = speed
        self.lineSegment = LineSegment(startPoint, endPoint)

    def draw(self, canvas, **kwargs):
        draw.drawLine(canvas, self.startPoint, self.endPoint, arrow=tk.LAST)

    def calcPointDebug(self, point):
        timeParametric = self.lineSegment.closestPointParametric(point)
        if timeParametric > 1.0:
            timeParametric = 1.0
        elif timeParametric < 0.0:
            timeParametric = 0.0
        closestPoint = self.lineSegment.getParametricPoint(timeParametric)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeParametric * self.time)

    def intersectsLine(self, obstacleLine):
        return obstacleLine.checkPathIntersectsLine(self.startPoint, self.endPoint, self.speed)
