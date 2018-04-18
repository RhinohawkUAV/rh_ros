import Tkinter as tk

import numpy as np

from engine.geometry import LineSegment
from gui import draw
from pathSegment import PathSegment


class LinePathSegment(PathSegment):
    def __init__(self, startPoint, speed, time, endPoint, endVelocity):
        PathSegment.__init__(self, time, endPoint, endVelocity)
        self.startPoint = startPoint
        self.speed = speed
        self.lineSegment = LineSegment(startPoint, endPoint)

    def draw(self, canvas, **kwargs):
        draw.drawLine(canvas, self.startPoint, self.endPoint, arrow=tk.LAST)

    def calcPointDebug(self, point):
        timeParametric = self.lineSegment.closestPointParametric(point)
        closestPoint = self.lineSegment.getParametricPoint(timeParametric)
        distance = np.linalg.norm(point - closestPoint)
        return (closestPoint, distance, timeParametric)

    def doesLineIntersect(self, obstacleLine, obstacleLineVelocity):
        """
        Does a path from startPoint to endPoint, at the given speed intersect the given lineSegment.
        """
        direction = self.endPoint - self.startPoint
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
        endPoint = self.startPoint + velocity * t

        return obstacleLine.checkLineIntersection(self.startPoint, endPoint)
