import Tkinter as tk

import numpy as np

from defaultPathSegment import DefaultPathSegment
from engine.geometry import LineSegment
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_DASH
from pathSegment import PathSegment

MAX_ARC_INTERPOLATION_ERROR = 4.0


class ArcPathSegment(DefaultPathSegment):
    def __init__(self, time, endPoint, endVelocity, speed, arc, arcTime):
        PathSegment.__init__(self, time, endPoint, endVelocity)
        self.speed = speed

        self.lineStartPoint = arc.endPoint
        self.lineSegment = LineSegment(arc.endPoint, endPoint)
        self.arc = arc
        self.arcTime = arcTime
        self.lineTime = self.time - self.arcTime
        self.linearPathPoints = arc.interpolate(MAX_ARC_INTERPOLATION_ERROR)
        self.linearPathPoints.append(endPoint)

    def draw(self, canvas, filtered=False, color=DEFAULT_COLOR, **kwargs):
        if filtered:
            dash = DEFAULT_DASH
        else:
            dash = None
        draw.drawLine(canvas, self.lineStartPoint, self.endPoint, color=color, arrow=tk.LAST, dash=dash)
        draw.drawArcObj(canvas, self.arc, color=color, dash=dash)
        for i in range(0, len(self.linearPathPoints) - 2):
            draw.drawLine(canvas, self.linearPathPoints[i], self.linearPathPoints[i + 1], color="orange", dash=dash)

    def calcPointDebug(self, point):
        linePointDebug = self.linePointDebug(point)
        arcPointDebug = self.arc.getPointDebug(point)
        if linePointDebug[1] < arcPointDebug[1]:
            pointDebug = linePointDebug
            startTime = self.arcTime
            segmentTime = self.lineTime
        else:
            pointDebug = arcPointDebug
            startTime = 0.0
            segmentTime = self.arcTime

        timeInterp = pointDebug[2]
        if timeInterp < 0.0:
            timeInterp = 0.0
        elif timeInterp > 1.0:
            timeInterp = 1.0
        time = startTime + timeInterp * segmentTime
        return (pointDebug[0], pointDebug[1], time)

    def linePointDebug(self, point):
        timeInterp = self.lineSegment.closestPointParametric(point)
        closestPoint = self.lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeInterp)

    def intersectsLine(self, obstacleLine):
        for i in range(0, len(self.linearPathPoints) - 1):
            if obstacleLine.checkPathIntersectsLine(startPoint=self.linearPathPoints[i],
                                                    endPoint=self.linearPathPoints[i + 1],
                                                    speed=self.speed):
                return True
        return False
