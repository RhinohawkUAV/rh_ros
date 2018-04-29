import Tkinter as tk

import numpy as np

from defaultPathSegment import DefaultPathSegment
from engine.geometry import LineSegment
from gui import draw
from pathSegment import PathSegment


class ArcPathSegment(DefaultPathSegment):
    def __init__(self, time, endPoint, endVelocity, speed, arc, arcTime):
        PathSegment.__init__(self, time, endPoint, endVelocity)
        self.speed = speed

        self.lineStartPoint = arc.endPoint
        self.lineSegment = LineSegment(arc.endPoint, endPoint)
        self.arc = arc
        self.arcTime = arcTime
        self.lineTime = self.time - self.arcTime

    def draw(self, canvas, **kwargs):
        draw.drawLine(canvas, self.lineStartPoint, self.endPoint, arrow=tk.LAST)
        # draw.drawArc(canvas, self.arcCenter, self.arcRadius, math.degrees(self.arcStart), math.degrees(self.arcLength))
        draw.drawArcObj(canvas, self.arc)

    def calcPointDebug(self, point):
        linePointDebug = self.linePointDebug(point)
        arcPointDebug = self.arcPointDebug(point)
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

    def arcPointDebug(self, point):
        return self.arc.getPointDebug(point)

    def linePointDebug(self, point):
        timeInterp = self.lineSegment.closestPointParametric(point)
        closestPoint = self.lineSegment.getParametricPoint(timeInterp)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeInterp)

    def intersectsLine(self, obstacleLine, obstacleLineVelocity):
        """
        Does a path from startPoint to endPoint, at the given speed intersect the given lineSegment.
        """
        direction = self.endPoint - self.lineStartPoint
        distance = np.linalg.norm(direction)
        if distance == 0.0:
            return False

        # velocity vector - has magnitude in speed heading in velocity from start to end
        velocity = (self.speed / distance) * direction

        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity -= obstacleLineVelocity

        # Time to get from start to end
        t = distance / self.speed

        # The new end point takes the same time to reach, but at a new offset heading
        endPoint = self.lineStartPoint + velocity * t

        return obstacleLine.checkLineIntersection(self.lineStartPoint, endPoint)
