import math

from engine.geometry import calcs
import numpy as np


class Arc:
    """
    Represents an arc for the purposes of path finding.  An arc can be CCW (rotDirection = 1) or CW (rotDirection = -1).  
    """

    def __init__(self, rotDirection, radius, center, start, length):
        self.rotDirection = rotDirection
        self.radius = radius
        self.center = center
        self.start = start
        self.length = length

    def getPointDebug(self, point):
        """
        Helper for standard point debug info.  For a given nearby point, this calculates:
        1. The closest point
        2. The distance to closest point
        3. The time interpolation along the arc (assumes constant speed currently).  This is a number [0,1]

        :param point: a nearby point to analyze
        :return: (closestPoint, distance, time interpolation)
        """
        fromCenterDir = point - self.center
        dirLength = np.linalg.norm(fromCenterDir)
        if dirLength == 0.0:
            # At center of circle we have to pick an arbitrary rotDirection which is closest
            fromCenterDir = np.array([1.0, 0.0], np.double)
        else:
            fromCenterDir /= dirLength

        pointAngle = calcs.angleOfVector(fromCenterDir, self.rotDirection)

        # Clamp the angle to be between start and end of the arc
        pointAngle = calcs.clampAngleCCW(pointAngle, self.start, self.length)

        if self.length == 0.0:
            timeInterp = 0.0
        else:
            angleDiff = calcs.modAngleUnsigned(pointAngle - self.start)
            timeInterp = angleDiff / self.length

        closestPoint = self.pointAtAngle(pointAngle)
        distance = np.linalg.norm(point - closestPoint)

        return (closestPoint, distance, timeInterp)

    def pointAtAngle(self, pointAngle):
        """
        Determine the point at the given angle.
        :param pointAngle:
        :return:
        """
        return self.center + self.radius * calcs.unitVectorOfAngle(pointAngle, self.rotDirection)

    def interpolate(self, maxError):
        """
        Return a series of points along the arc for linear interpolation.  The maximum error (distance from a line
        segment to a true point on the arc) defines the number of points in the interpolation.
        :param maxError: MUST BE > 0.0!  If this exceeds radius the result will be single line interpolation whose error is radius.
        :return:
        """

        # If the arc has no length just return 1 point at the start of the arc
        if self.length == 0.0 or self.radius == 0.0:
            return [self.pointAtAngle(self.start)]

        if maxError > self.radius:
            maxError = self.radius
        maxAngle = math.acos(1.0 - maxError / self.radius)
        numLines = int(math.ceil(self.length / maxAngle))

        step = self.length / numLines
        points = []

        for i in range(0, numLines + 1):
            point = self.pointAtAngle(self.start + i * step)
            points.append(point)
        return points
    
