import Tkinter as tk
from defaultPathSegment import DefaultPathSegment
from engine.geometry import LineSegment
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_DASH, DEFAULT_WIDTH
import numpy as np
from pathSegment import PathSegment


class LinePathSegment(DefaultPathSegment):

    def __init__(self, startTime, startPoint, startSpeed, elapsedTime, endPoint, endSpeed, endUnitVelocity):
        PathSegment.__init__(self, startTime, elapsedTime, endPoint, endSpeed, endUnitVelocity)
        self.startPoint = startPoint
        self.startSpeed = startSpeed
        self.lineSegment = LineSegment(startPoint, endPoint)

    def draw(self, canvas, color=DEFAULT_COLOR, filtered=False, width=DEFAULT_WIDTH, **kwargs):
        if filtered:
            dash = DEFAULT_DASH
        else:
            dash = None
        draw.drawLine(canvas, self.startPoint, self.endPoint, color=color, arrow=tk.LAST, dash=dash, width=width)

    def calcPointDebug(self, point):
        timeInterp = self.lineSegment.closestPointParametric(point)
        if timeInterp > 1.0:
            timeInterp = 1.0
        elif timeInterp < 0.0:
            timeInterp = 0.0
        closestPoint = self.lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, self.startTime + timeInterp * self.elapsedTime)

    def intersectsObstacleLine(self, startTime, obstacleLine):
        return obstacleLine.checkPathIntersectsLine(startTime, self.startPoint, self.endPoint, self.startSpeed)
