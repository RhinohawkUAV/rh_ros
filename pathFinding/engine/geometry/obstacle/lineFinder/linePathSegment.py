import Tkinter as tk
from engine.geometry import LineSegment
from engine.geometry.obstacle.pathSegment import PathSegment
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_DASH, DEFAULT_WIDTH
import numpy as np


class LinePathSegment(PathSegment):

    def __init__(self, startTime, startPoint, startSpeed, elapsedTime, endPoint, endSpeed, endUnitVelocity, nextLegalRotDirection):
        PathSegment.__init__(self, startTime, elapsedTime, endPoint, endSpeed, endUnitVelocity, nextLegalRotDirection)
        self.startPoint = startPoint
        self.startSpeed = startSpeed
        self.lineSegment = LineSegment(startPoint, endPoint)

    def draw(self, visualizer, color=DEFAULT_COLOR, filtered=False, width=DEFAULT_WIDTH, **kwargs):
        if filtered:
            dash = DEFAULT_DASH
        else:
            dash = None
        draw.drawLine(visualizer, self.startPoint, self.endPoint, color=color, arrow=tk.LAST, dash=dash, width=width)

    def calcPointDebug(self, point):
        timeInterp = self.lineSegment.closestPointParametric(point)
        if timeInterp > 1.0:
            timeInterp = 1.0
        elif timeInterp < 0.0:
            timeInterp = 0.0
        closestPoint = self.lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, self.startTime + timeInterp * self.elapsedTime)

    def testIntersection(self, pathIntersectionDetector):
        return pathIntersectionDetector.testStraightPathIntersection(self.startTime, self.startPoint, self.endSpeed * self.endUnitVelocity, self.elapsedTime)
    
