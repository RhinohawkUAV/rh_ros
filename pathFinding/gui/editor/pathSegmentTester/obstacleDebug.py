import Tkinter as tk
from gui import draw
from gui.draw import DEFAULT_COLOR
import numpy as np


class NoFlyZoneDebug:

    def __init__(self, noFlyZoneInput):
        # Points composing the border of the NFZ
        self._points = np.array(noFlyZoneInput.points, np.double)
        self._velocity = np.array(noFlyZoneInput.velocity, np.double)
        self._midPoint = self._points.sum(axis=0) / len(self._points)

    def draw(self, canvas, time=0.0, color=DEFAULT_COLOR, **kwargs):
        offset = self._velocity * time
        for i in range(len(self._points)):
            draw.drawLine(canvas, self._points[i - 1] + offset, self._points[i] + offset, color=color)

        if np.linalg.norm(self._velocity) > 0.0:
            draw.drawVelocity(canvas, self._midPoint + offset, self._velocity, **kwargs)


class ObstacleCourseDebug:
    """
    Class for visual tracking/displaying information about the boundary and no fly zones over time.
    """

    def __init__(self, boundaryPoints, noFlyZones):
        self.boundaryPoints = boundaryPoints
        self.noFlyZonesDebug = []
        for noFlyZoneInput in noFlyZones:
            self.noFlyZonesDebug.append(NoFlyZoneDebug(noFlyZoneInput))

    def draw(self, canvas, time=0.0, boundaryColor=DEFAULT_COLOR, nfzColor=DEFAULT_COLOR, **kwargs):
        for i in range(len(self.boundaryPoints)):
            draw.drawLine(canvas, self.boundaryPoints[i - 1], self.boundaryPoints[i], color=boundaryColor)
        for noFlyZoneDebug in self.noFlyZonesDebug:
            noFlyZoneDebug.draw(canvas, time=time, color=nfzColor)
