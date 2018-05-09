import Tkinter as tk
from engine import PointToPointInput
from engine.geometry import calcs
from gui import Drawable, draw
from gui.draw import DEFAULT_POINT_SIZE, DEFAULT_COLOR
import numpy as np

# TODO: Offset will not look correct for other scalings, similar problem for DEFAULT_POINT_SIZE.
# Ideally this will be a non-scaling factor.
TEXT_OFFSET = np.array((2, 0), np.double)


class PathEdit(Drawable):

    def __init__(self):
        self.points = []
        self._startVelocity = np.array((0, 0), np.double)

    def setToInput(self, input):
        del self.points[:]
        self.points.append(input.startPosition)
        self.points.extend(input.targetPoints)
        self._startVelocity = np.array(input._startVelocity, np.double)

    def findClosestPoint(self, point):
        return calcs.findClosestPoint(point, self.points)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        for i in range(len(self.points)):
            point = self.points[i]
            draw.drawPoint(canvas, point, radius=radius, color=color)
            draw.drawText(canvas, point + TEXT_OFFSET, str(i), color=color)

        if len(self.points) > 0:
            draw.drawLine(canvas, self.points[0],
                          self.points[0] + self._startVelocity,
                          arrow=tk.LAST)

    def toJSONDict(self):
        # If this has not been setup, then return None, which translates to null in JSON
        if len(self.points) < 2:
            return None
        return PointToPointInput(self.points[0], self._startVelocity, self.points[1:]).__dict__
