import math
from Tkinter import Canvas

import numpy as np

import gui.draw
from gui import Drawable

textOffsetFactor = 4.0
normalDisplayFactor = 4.0


class LineSegment(Drawable):
    """
    A one-sided line segment.  This allows collision detection with another directed line-segment.  A collision is
    only considered if the velocity of the other line segment opposes the normal of this line segment.
    Normal is calculated such that if a series of these defines a polygon using counter-clockwise winding, all normals
    would face outward.  The polygon could then detect collisions with line segments directed inwards, but not outward.

    This will be converted to C or a collection of line segments stored in a numpy array.
    """

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

        # Points from point 1 to point 2 with length inverse to the distance between the _points.
        if magSquared == 0.0:
            pass
        self.invTan = diff / magSquared

        # For drawing ONLY
        self.mid = (self.p1 + self.p2) / 2.0

    def pointSignedDistance(self, point):
        p1diff = point - self.p1
        return np.dot(self.n, p1diff)

    def pointDistance(self, point):
        p1diff = point - self.p1
        return math.fabs(np.dot(self.n, p1diff))

    def projectPoint(self, point):
        # Distance to one-sided line segment, from start, in the direction of the normal.
        p1diff = point - self.p1
        normalDistance = np.dot(self.n, p1diff)
        return point - (self.n * normalDistance)

    def closestPointParametric(self, point):
        projectedPoint = self.projectPoint(point)
        parametric = np.dot(projectedPoint - self.p1, self.invTan)
        return parametric

    def getParametricPoint(self, parametric):
        if parametric < 0.0:
            return self.p1
        elif parametric > 1.0:
            return self.p2
        else:
            return (1.0 - parametric) * self.p1 + parametric * self.p2

    def closestPoint(self, point):
        parametric = self.closestPointParametric(point)
        return self.getParametricPoint(parametric)

    def checkLineIntersection(self, start, end):
        """
        TODO: Consider how we want to handle edge cases and tolerance

        Determine if the directed line segment, from p1 to p2, intersects this one-sided line segment.

        Only considered an intersection if velocity from start to end is opposed the normal vector of the line.
        If the velocity from start to end is parallel, then it does not count.
        Intersection with an endpoint of this one-sided line segment does count.

        These decisions more or less give us the desired outcomes when considering tracings around the outside of a polygon.
        The only bad case is when parallel segments of different polygons align exactly.
        We may want to add a tolerance to be sure that we can't ever pick a line exactly through 2 endpoints of a line segments in a polygon

        :param start: (x,y) 1st point of the line
        :param end: (x,y) 2nd point of the line
        :return: is there an intersection
        """

        # Distance to one-sided line segment, from start, in the direction of the normal.
        p1diff = start - self.p1
        normalDistanceP1 = np.dot(self.n, p1diff)

        # If this is negative then start is "behind" the one-sided line-segment and no intersection is possible.
        if normalDistanceP1 < 0.0:
            return False

        # Distance to one-sided line segment, from end, in the direction of the normal.
        p2diff = end - self.p1
        normalDistanceP2 = np.dot(self.n, p2diff)

        # If this is positive then end is "in front of" the one-sided line-segment and no intersection is possible.
        if normalDistanceP2 > 0.0:
            return False

        # The diff vector of the line segment
        lineDir = end - start

        # Equivalent of: normalDirection = dot(n,lineDir).  We don't normalize as the magnitude of this cancels out with itself later.
        normalDirection = normalDistanceP2 - normalDistanceP1

        # As you move from start towards end you approach intersection with the infinite line defined by the one-sided
        # line segment. The crossing point will be at start + lineDir * t, for some t.
        # t is just the ratio below:
        t = -normalDistanceP1 / normalDirection

        # While moving towards the line perpendicularly, we also moved along the line tangentially.
        # We don't care how far we moved in absolute space, only in "tangent-unit-space".  In this space start is 0, end is 1.
        # The provided invTan vector _points in the velocity from start to end with the inverse magnitude of the length between them.
        parametric = np.dot(p1diff + lineDir * t, self.invTan)

        return parametric >= 0 and parametric <= 1

    def draw(self, canvas, text="", time=0.0, drawVectors=True, **kwargs):
        """
        Draw on the canvas with any modifiers stored in kwargs.
        :param canvas:
        :param kwargs:
        :return:
        """

        # type: (Canvas, str) -> None
        if not text == "":
            textPos = self.mid - self.n * textOffsetFactor
            gui.draw.drawText(canvas, textPos, text=text, **kwargs)

        gui.draw.drawLine(canvas, self.p1, self.p2, **kwargs)
        if drawVectors:
            gui.draw.drawLine(canvas, self.mid, self.mid + self.n * normalDisplayFactor, **kwargs)

    def xRay(self, point):
        """
        Used only by editor tool
        Used when computing if a point is inside a polygon.  This tests whether a ray from point in the +x direction
        would intersect the line.
        :param point:
        :return:
        """

        # Horizontal
        if self.n[0] == 0.0:
            return False

        if (point[1] > self.p1[1] and point[1] <= self.p2[1]) or (
                point[1] > self.p2[1] and point[1] <= self.p1[1]):
            if self.invTan[0] == 0.0:
                x = self.p1[0]
            else:
                x = self.p1[0] + (self.invTan[0] * (point[1] - self.p1[1]) / self.invTan[1])
            return point[0] < x
        else:
            return False
