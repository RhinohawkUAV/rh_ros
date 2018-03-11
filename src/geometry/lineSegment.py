from Tkinter import Canvas

import numpy as np

from geometry import intersection
from gui import Drawable

textOffsetFactor = 4.0
normalDisplayFactor = 4.0


class LineSeg(Drawable):
    def __init__(self, p1, p2):
        # Point 1
        self.p1 = np.array(p1, np.double)

        # Point 2
        self.p2 = np.array(p2, np.double)

        diff = self.p2 - self.p1
        magSquared = diff[0] * diff[0] + diff[1] * diff[1]

        # Unit normal of the line.  Paths only intersect this line if opposing the normal.
        # Normal is chosen so that, if polygons are wound counter-clockwise,
        # normals point outwards.
        self.n = np.array([-diff[1], diff[0]], np.double)
        self.n /= np.math.sqrt(magSquared)

        # Points from point 1 to point 2 with length inverse to the distance between the points.
        self.invTan = diff / magSquared

        # For drawing ONLY
        self.mid = (self.p1 + self.p2) / 2.0

    def checkIntersection(self, rayStart, rayDir):
        return intersection.checkRayIntersectLine(rayStart, rayDir, self.p1, self.p2, self.n, self.invTan)

    def draw(self, canvas, text="", **kwargs):
        """
        Draw on the canvas with any modifiers stored in kwargs.
        :param canvas:
        :param kwargs:
        :return:
        """

        # type: (Canvas, str) -> None
        if not text == "":
            textPos = self.mid - self.n * textOffsetFactor
            canvas.create_text(textPos[0], textPos[1], text=text, fill="black")

        drawLine(canvas, self.p1, self.p2,**kwargs)
        drawLine(canvas, self.mid, self.mid + self.n * normalDisplayFactor,**kwargs)


def drawLine(canvas, p1, p2, **kwargs):
    canvas.create_line(p1[0], p1[1], p2[0], p2[1], **kwargs)
