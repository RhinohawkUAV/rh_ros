import Tkinter as tk
from constants import MAX_ARC_INTERPOLATION_ERROR
from engine.geometry import LineSegment, calcs
from engine.geometry.obstacle.pathSegment import PathSegment
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_DASH, DEFAULT_WIDTH
import numpy as np


class ArcPathSegment(PathSegment):

    def __init__(self, startTime, elapsedTime, lineStartPoint, endPoint, endSpeed, endUnitVelocity, arc):
        PathSegment.__init__(self, startTime, elapsedTime, endPoint, endSpeed, endUnitVelocity)
        self.speed = endSpeed

        self.lineStartPoint = lineStartPoint
        self.arc = arc
        self.arcTime = self.arc.length * self.arc.radius / self.speed
        self.lineTime = self.elapsedTime - self.arcTime

        self.linearPathPoints = arc.interpolate(MAX_ARC_INTERPOLATION_ERROR)
        numArcPoints = len(self.linearPathPoints)
        self.linearPathTimes = []
        if numArcPoints == 1:
                self.linearPathTimes.append(startTime)
        else:
            for i in range(numArcPoints):
                self.linearPathTimes.append(self.startTime + i * self.arcTime / (numArcPoints - 1))

        if self.lineTime > 0.0:
            self.linearPathPoints.append(endPoint)
            self.linearPathTimes.append(self.startTime + self.elapsedTime)
        
    def draw(self, canvas, filtered=False, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, **kwargs):
        if filtered:
            dash = DEFAULT_DASH
        else:
            dash = None
        draw.drawLine(canvas, self.lineStartPoint, self.endPoint, color=color, arrow=tk.LAST, dash=dash, width=width)
        draw.drawArcObj(canvas, self.arc, color=color, dash=dash, width=width)
        for i in range(0, len(self.linearPathPoints) - 2):
            draw.drawLine(canvas, self.linearPathPoints[i], self.linearPathPoints[i + 1], color="orange", dash=dash)

    def calcPointDebug(self, point):
        linePointDebug = self.linePointDebug(point)
        arcPointDebug = self.arc.getPointDebug(point)
        if linePointDebug[1] < arcPointDebug[1]:
            pointDebug = linePointDebug
            segmentStartTime = self.arcTime
            segmentTimeElapsed = self.lineTime
        else:
            pointDebug = arcPointDebug
            segmentStartTime = 0.0
            segmentTimeElapsed = self.arcTime

        timeInterp = pointDebug[2]
        if timeInterp < 0.0:
            timeInterp = 0.0
        elif timeInterp > 1.0:
            timeInterp = 1.0
        time = self.startTime + segmentStartTime + timeInterp * segmentTimeElapsed
        return (pointDebug[0], pointDebug[1], time)

    def linePointDebug(self, point):
        lineSegment = LineSegment(self.lineStartPoint, self.endPoint)
        timeInterp = lineSegment.closestPointParametric(point)
        closestPoint = lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeInterp)

    def testIntersection(self, pathIntersectionDetector):
        return pathIntersectionDetector.testStraightPathIntersections(self.linearPathPoints, self.linearPathTimes)

