import math

from engine.geometry import calcs
import numpy as np

    
def createArc(startPoint, speed, unitVelocity, acceleration, rotDirection):
    radius = speed * speed / acceleration
    fromCenterDir = -rotDirection * calcs.CCWNorm(unitVelocity)
    fromCenterToStart = fromCenterDir * radius

    center = startPoint - fromCenterToStart
    start = calcs.angleOfVector(fromCenterToStart, rotDirection)

    length = 0.0

    return Arc(rotDirection, radius, center, start, length)


class Arc:
    """
    Represents an arc for the purposes of path finding.  An arc can be CCW (rotDirection = 1) or CW (rotDirection = -1).  
    The initial length of the arc is 0.0 and can be changed by calling the setLength() method.

    Once setup a number of useful queries can be made:
    1. endPoint and endTangent (position and direction of travel at the end of the arc of given length)
    2. point debug which gets the closest point on the arc, to a given point, along with distance and time interpolation for display.
    3. an interpolation into line segments for collision detection
    """

    def __init__(self, rotDirection, radius, center, start, length):
        self.rotDirection = rotDirection
        self.radius = radius
        self.center = center
        self.start = start
        self.setLength(length)

    def setLength(self, length):
        """
        Set the arc's length and calculate a new end point/tangent as well
        :param length:
        :return:
        """
        self.length = length
        endAngle = self.start + self.length
        arcEnd = calcs.unitVectorOfAngle(endAngle, self.rotDirection)

        self.endTangent = self.rotDirection * calcs.CCWNorm(arcEnd)
        self.endPoint = self.center + self.radius * arcEnd

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
        :param maxError: cannot exceed radius (the result will be single line interpolation whose error is radius).
        :return:
        """

        # TODO: Check various div 0s
        # If the arc has no length just return 2 points at the start of the arc
        if self.length == 0.0:
            startPoint = self.pointAtAngle(self.start)
            return [startPoint, startPoint]

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
