import math

import numpy as np

from engine.geometry import calcs


class Arc:
    """
    Represents an arc for the purposes of path finding.  An arc is created based on a starting position, velocity and
    acceleration.  An arc can be CCW (direction = 1) or CW (direction = -1).  The initial length of the arc is 0.0 and
    can be changed by calling the setLength() method.

    Once setup a number of useful queries can be made:
    1. endPoint and endTangent (position and direction of travel at the end of the arc of given length)
    2. point debug which gets the closest point on the arc, to a given point, along with distance and time interpolation for display.
    3. an interpolation into line segments for collision detection
    """

    def __init__(self, startPoint, startVelocity, acceleration, direction):
        self.direction = direction
        speed = np.linalg.norm(startVelocity)
        startDirection = startVelocity / speed
        self.radius = speed * speed / acceleration
        fromCenterDir = -self.direction * calcs.CCWNorm(startDirection)
        fromCenterToStart = fromCenterDir * self.radius

        self.center = startPoint - fromCenterToStart
        self.start = math.atan2(fromCenterToStart[1], fromCenterToStart[0])
        self.length = 0.0
        self.endPoint = startPoint
        self.endTangent = startDirection

    def setLength(self, length):
        """
        Set the arc's length and calculate a new end point/tangent as well
        :param length:
        :return:
        """
        self.length = length
        endAngle = self.start + self.length * self.direction
        arcEnd = np.array([math.cos(endAngle), math.sin(endAngle)])
        self.endTangent = self.direction * calcs.CCWNorm(arcEnd)
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
        dir = point - self.center
        dirLength = np.linalg.norm(dir)
        if dirLength == 0.0:
            # At center of circle we have to pick an arbitrary direction which is closest
            dir = np.array([1.0, 0.0], np.double)
        else:
            dir /= dirLength

        pointAngle = math.atan2(dir[1], dir[0])

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
        return self.center + self.radius * np.array([math.cos(pointAngle), math.sin(pointAngle)])

    def interpolate(self, maxError):
        """
        Return a series of points along the arc for linear interpolation.  The maximum error (distance from a line
        segment to a true point on the arc) defines the number of points in the interpolation.
        :param maxError: cannot exceed radius (the result will be single line interpolation whose error is radius).
        :return:
        """

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
