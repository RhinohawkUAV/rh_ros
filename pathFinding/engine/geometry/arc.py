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


def createArc(startPoint, startVelocity, acceleration, rotationDirection):
    speed = np.linalg.norm(startVelocity)
    direction = startVelocity / speed
    arcRadius = speed * speed / acceleration
    fromCenterDir = -rotationDirection * calcs.CCWNorm(direction)
    fromCenterToStart = fromCenterDir * arcRadius
    arcCenter = startPoint - fromCenterToStart
    arcStartAngle = math.atan2(fromCenterToStart[1], fromCenterToStart[0])
    return Arc(arcCenter, arcRadius, arcStartAngle, 0.0)
