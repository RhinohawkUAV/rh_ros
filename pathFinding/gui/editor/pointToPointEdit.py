import Tkinter as tk

import numpy as np

from engine import PointToPointInput
from engine.geometry import calcs
from gui import Drawable, draw
# TODO: Offset will not look correct for other scalings, similar problem for DEFAULT_POINT_SIZE.
# Ideally this will be a non-scaling factor.
from gui.draw import DEFAULT_POINT_SIZE, DEFAULT_COLOR

TEXT_OFFSET = np.array((2, 0), np.double)


class PointToPointEdit(Drawable):
    def __init__(self):
        self.points = []
        self.startVelocity = np.array((0, 0), np.double)

    def setToInput(self, input):
        del self.points[:]
        self.points.append(input.startPosition)
        self.points.extend(input.targetPoints)
        self.startVelocity = np.array(input.startVelocity, np.double)

    def findClosestPoint(self, point):
        return calcs.findClosestPoint(point, self.points)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR,**kwargs):
        for i in range(len(self.points)):
            point = self.points[i]
            draw.drawPoint(canvas, point, radius=radius, color=color)
            draw.drawText(canvas, point + TEXT_OFFSET, str(i), color=color)

        if len(self.points) > 0:
            draw.drawLine(canvas, self.points[0],
                          self.points[0] + self.startVelocity,
                          arrow=tk.LAST)

    def toJSONDict(self):
        # If this has not been setup, then return None, which translates to null in JSON
        if len(self.points) < 2:
            return None
        return PointToPointInput(self.points[0], self.startVelocity, self.points[1:]).__dict__
