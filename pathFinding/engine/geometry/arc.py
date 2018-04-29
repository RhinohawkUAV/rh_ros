import math

import numpy as np

from engine.geometry import calcs


class Arc:
    def __init__(self, center, radius, start, length=None, direction=1):
        self.center = center
        self.radius = radius
        self.start = start
        self.direction = direction
        if length is not None:
            self.setLength(length)
        else:
            self.length = None
            self.endPoint = None
            self.endTangent = None

    def setLength(self, length):
        self.length = length
        endAngle = self.start + self.length * self.direction
        arcEnd = np.array([math.cos(endAngle), math.sin(endAngle)])
        self.endTangent = self.direction * calcs.CCWNorm(arcEnd)
        self.endPoint = self.center + self.radius * arcEnd

    def getPointDebug(self, point):
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
        return self.center + self.radius * np.array([math.cos(pointAngle), math.sin(pointAngle)])

    def interpolate(self, maxError):
        """
        Return a series of points along the arc for linear interpolation.  The maximum error (distance from a line
        segment to a true point on the arc) defines how closely to perform the interpolation.
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


def createArc(startPoint, startVelocity, acceleration, rotationDirection):
    speed = np.linalg.norm(startVelocity)
    direction = startVelocity / speed
    arcRadius = speed * speed / acceleration
    fromCenterDir = -rotationDirection * calcs.CCWNorm(direction)
    fromCenterToStart = fromCenterDir * arcRadius
    arcCenter = startPoint - fromCenterToStart
    arcStartAngle = math.atan2(fromCenterToStart[1], fromCenterToStart[0])
    return Arc(arcCenter, arcRadius, arcStartAngle, 0.0)
