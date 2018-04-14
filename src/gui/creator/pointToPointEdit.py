import Tkinter as tk

import numpy as np

from findPath import PointToPointInput
from findPath.geometry import calcs
from gui import Drawable, draw
# TODO: Offset will not look correct for other scalings, similar problem for DEFAULT_POINT_SIZE.
# Ideally this will be a non-scaling factor.
from gui.draw import DEFAULT_POINT_SIZE, DEFAULT_COLOR

TEXT_OFFSET = np.array((2, 0), np.double)


class PointToPointEdit(Drawable):
    def __init__(self):
        self.points = []
        self.initialVelocity = np.array((0, 0), np.double)

    def findClosestPoint(self, point):
        return calcs.findClosestPoint(point, self.points)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR):
        for i in range(len(self.points)):
            point = self.points[i]
            draw.drawPoint(canvas, point, radius=radius, color=color)
            draw.drawText(canvas, point + TEXT_OFFSET, str(i), color=color)

        if len(self.points) > 0:
            draw.drawLine(canvas, self.points[0],
                          self.points[0] + self.initialVelocity,
                          arrow=tk.LAST)

    def toInput(self):
        return PointToPointInput(self.points[0], self.initialVelocity, self.points[1:])
