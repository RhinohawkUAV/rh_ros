from engine import NoFlyZone
from engine.geometry import LineSegment, calcs
from gui import Drawable
import gui.draw
import numpy as np


class EditableNoFlyZone(Drawable):

    def __init__(self, points, velocity):
        """
        A polygon NFZ with a given velocity.
        :param points:
        :param velocity:
        """

        self.points = np.array(points, np.double)
        self.velocity = np.array(velocity, np.double)

        self._midPoint = self.points.sum(axis=0) / len(self.points)
        self._lines = []

        for i in range(0, len(points)):
            self._lines.append(LineSegment(self.points[i - 1], self.points[i]))

    def draw(self, canvas, **kwargs):
        # type: (Canvas) -> None

        for line in self._lines:
            line.draw(canvas, **kwargs)

        if np.linalg.norm(self.velocity) > 0.0:
            gui.draw.drawVelocity(canvas, self._midPoint, self.velocity, **kwargs)

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
        return EditableNoFlyZone(points, self.velocity)

    def getPointTranslatedCopy(self, pointIndex, newPoint):
        """Create a copy of this NFZ, with the given point replaced with the given point"""
        points = np.copy(self.points)
        points[pointIndex] = newPoint
        return EditableNoFlyZone(points, self.velocity)

    def asInput(self, ID):
        return NoFlyZone(self.points, self.velocity, ID)


def fromInput(noFlyZoneInput):
    return EditableNoFlyZone(noFlyZoneInput.points, noFlyZoneInput.velocity)


def listFromInput(noFlyZoneInputList):
    noFlyZones = []
    for noFlyZoneInput in noFlyZoneInputList:
        noFlyZones.append(fromInput(noFlyZoneInput))
    return noFlyZones
