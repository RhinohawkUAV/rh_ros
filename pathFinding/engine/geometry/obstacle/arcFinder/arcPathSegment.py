import Tkinter as tk
from constants import MAX_ARC_INTERPOLATION_ERROR
from engine.geometry import LineSegment
from engine.geometry.obstacle.pathSegment import PathSegment
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_DASH, DEFAULT_WIDTH
import numpy as np


class ArcPathSegment(PathSegment):

    def __init__(self, startTime, elapsedTime, lineStartPoint, endPoint, endSpeed, endUnitVelocity, arc):
        PathSegment.__init__(self, startTime, elapsedTime, endPoint, endSpeed, endUnitVelocity)
        self.speed = endSpeed

        self.lineStartPoint = lineStartPoint
        self.lineSegment = LineSegment(self.lineStartPoint, endPoint)
        self.arc = arc
        self.arcTime = self.arc.length * self.arc.radius / self.speed
        self.lineTime = self.elapsedTime - self.arcTime

        self.linearPathPoints = arc.interpolate(MAX_ARC_INTERPOLATION_ERROR)
        numArcSegments = len(self.linearPathPoints) - 1

        self.linearPathPoints.append(endPoint)
        
        # Remember the start times for each path segment to individually query for collisions
        self.linearPathStartTimes = []
        for i in range(numArcSegments):
            startTime = i * self.arcTime / numArcSegments
            self.linearPathStartTimes.append(startTime)
        self.linearPathStartTimes.append(self.arcTime)

        # TODO: Check distance ==0 (can we use time ==0)?
        
        # TODO: Should be organized better.  Perhaps computed in collision detection system.
        self.linearPathTimes = []
        self.linearPathVelocities = []
        for i in range(len(self.linearPathStartTimes) - 1):
            time = self.linearPathStartTimes[i + 1] - self.linearPathStartTimes[i]
            self.linearPathTimes.append(time)
            self.linearPathVelocities.append((self.linearPathPoints[i + 1] - self.linearPathPoints[i]) / time)
        self.linearPathTimes.append(self.lineTime)
        self.linearPathVelocities.append(endSpeed * endUnitVelocity)
        
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
        timeInterp = self.lineSegment.closestPointParametric(point)
        closestPoint = self.lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeInterp)

    def testIntersection(self, pathIntersectionDetector):
        for i in range(0, len(self.linearPathPoints) - 1):
            if pathIntersectionDetector.testStraightPathIntersection(self.startTime + self.linearPathStartTimes[i],
                                                    startPoint=self.linearPathPoints[i],
                                                    velocity=self.linearPathVelocities[i],
                                                    time=self.linearPathTimes[i]):
                return True
        return False

