import Tkinter as tk
import math

import numpy as np

from defaultPathSegment import DefaultPathSegment
from engine.geometry import LineSegment
from gui import draw
from pathSegment import PathSegment


class ArcPathSegment(DefaultPathSegment):
    def __init__(self, time, endPoint, endVelocity, speed, lineStartPoint, arcStart, arcLength, arcCenter, arcRadius,
                 arcTime):
        PathSegment.__init__(self, time, endPoint, endVelocity)
        self.speed = speed

        self.lineStartPoint = lineStartPoint
        self.lineSegment = LineSegment(lineStartPoint, endPoint)

        self.arcStart = arcStart
        self.arcLength = arcLength
        self.arcCenter = arcCenter
        self.arcRadius = arcRadius
        self.arcTime = arcTime
        self.lineTime = self.time - self.arcTime

    def draw(self, canvas, **kwargs):
        draw.drawLine(canvas, self.lineStartPoint, self.endPoint, arrow=tk.LAST)
        draw.drawArc(canvas, self.arcCenter, self.arcRadius, math.degrees(self.arcStart), math.degrees(self.arcLength))

    def calcPointDebug(self, point):
        timeParametric = self.lineSegment.closestPointParametric(point)
        if timeParametric > 1.0:
            timeParametric = 1.0
        elif timeParametric < 0.0:
            timeParametric = 0.0
        closestPoint = self.lineSegment.getParametricPoint(timeParametric)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeParametric * self.lineTime + self.arcTime)

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
