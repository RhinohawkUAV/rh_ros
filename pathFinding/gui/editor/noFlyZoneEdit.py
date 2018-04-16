import Tkinter as tk
from Tkinter import Canvas

import numpy as np

import gui.draw
from engine import NoFlyZoneInput
from engine.geometry import LineSegment, calcs
from gui import Drawable


class NoFlyZoneEdit(NoFlyZoneInput, Drawable):
    def __init__(self, points, velocity):
        """
        A polygon NFZ with a given velocity.  Points must be given in CCW order.
        :param points:
        :param velocity:
        """
        NoFlyZoneInput.__init__(self, np.array(points, np.double), np.array(velocity, np.double))

        self._midPoint = self.points.sum(axis=0) / len(self.points)
        self._lines = []

        for i in range(0, len(points)):
            self._lines.append(LineSegment(self.points[i - 1], self.points[i]))

    def draw(self, canvas, **kwargs):
        # type: (Canvas) -> None

        for line in self._lines:
            line.draw(canvas, drawVectors=False, **kwargs)

        if np.linalg.norm(self.velocity) > 0.0:
            gui.draw.drawLine(canvas, self._midPoint, self._midPoint + self.velocity,
                              arrow=tk.LAST, **kwargs)

    def isPointInside(self, point):
        """
        Tests if a point is inside the NFZ.

        :param point:
        :return:
        """
        i = 0
        for line in self._lines:
            if line.xRay(point):
                i += 1
        return (i % 2) == 1

    def findClosestPoint(self, point):
        return calcs.findClosestPoint(point, self.points)

    def getTranslatedCopy(self, translation):
        """Create a copy of this NFZ, at a translated position."""
        points = []
        for point in self.points:
            points.append(point + translation)
        return NoFlyZoneEdit(points, self.velocity)

    def getPointTranslatedCopy(self, pointIndex, newPoint):
        """Create a copy of this NFZ, with the given point replaced with the given point"""
        points = np.copy(self.points)
        points[pointIndex] = newPoint
        return NoFlyZoneEdit(points, self.velocity)

    def toInput(self, ID):
        return NoFlyZoneInput(self.points, self.velocity, ID)


def fromInput(noFlyZoneInput):
    return NoFlyZoneEdit(noFlyZoneInput.points, noFlyZoneInput.velocity)


def listFromInput(noFlyZoneInputList):
    noFlyZones = []
    for noFlyZoneInput in noFlyZoneInputList:
        noFlyZones.append(fromInput(noFlyZoneInput))
    return noFlyZones
