import math

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment

MAX_ITERATIONS = 4

# There will be difference between an arc's exit velocity and the correct velocity vector.
# This represents the allowed angular error (in radians) and corresponds to missing a target
# 1km away by 1m
MAX_ANGLE_ERROR = 0.001

MIN_ANGLE_ERROR_COS = math.cos(MAX_ANGLE_ERROR)


def relativeAngleCCW(startVec, endVec):
    """
    How far you would have to turn, CCW, to go from startVec to endVec.  This will be in the range (-2*pi, 2*pi).
    :param startVec:
    :param endVec:
    :return:
    """

    return math.atan2(endVec[1], endVec[0]) - math.atan2(startVec[1], startVec[0])

    # sinRotate = startVec[0] * endVec[1] - startVec[1] * endVec[0]
    # if np.dot(startVec, endVec) < 0.0:
    #     return math.pi - math.asin(sinRotate)
    # else:
    #     return math.asin(sinRotate)


def modAngle(angle, lowAngle):
    """
    Return angle in the range: [lowAngle, 2*pi+lowAngle)
    :param angle:
    :param lowAngle:
    :return:
    """
    if angle < lowAngle:
        return angle + 2.0 * math.pi
    elif angle >= lowAngle + 2.0 * math.pi:
        return angle - 2.0 * math.pi
    else:
        return angle


def modAngleSigned(angle):
    """
    Return angle in the range: [-pi,pi)
    :param angle:
    :return:
    """
    return modAngle(angle, -math.pi)


def modAngleUnsigned(angle):
    """
    Return angle in the range: [0,2*pi)
    :param angle:
    :return:
    """
    return modAngle(angle, 0)


class ArcObstacleData(DefaultObstacleData):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, targetOffsetLength=NO_FLY_ZONE_POINT_OFFSET):
        DefaultObstacleData.__init__(self, targetOffsetLength)

    def createPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        arcFinder = ArcFinder(startPoint, startVelocity, targetPoint, velocityOfTarget, 1.0)
        try:
            arcFinder.solve()
            return ArcPathSegment(arcFinder.totalTime, arcFinder.endPoint, arcFinder.finalVelocity,
                                  arcFinder.speed, arcFinder.arcEndPoint, arcFinder.arcStartAngle, arcFinder.arcLength,
                                  arcFinder.arcCenter, arcFinder.arcRadius,
                                  arcFinder.arcTime)
        except:
            return None


class NoSolutionException(object):
    pass


class ArcFinder:

    def __init__(self, startPoint, startVelocity, targetPoint, velocityOfTarget, rotationDirection=1.0,
                 acceleration=1.0):
        self.startPoint = startPoint
        self.startVelocity = startVelocity
        self.speed = np.linalg.norm(startVelocity)
        self.startDirection = startVelocity / self.speed
        self.targetPoint = targetPoint
        self.velocityOfTarget = velocityOfTarget

        # Radius of the arc
        self.arcRadius = self.speed * self.speed / acceleration
        fromCenterDir = -rotationDirection * calcs.CCWNorm(self.startDirection)
        self.fromCenter = fromCenterDir * self.arcRadius
        self.arcCenter = self.startPoint - self.fromCenter
        self.arcStartAngle = math.atan2(self.fromCenter[1], self.fromCenter[0])

        self.arcExitDirection = self.startDirection
        self.totalTime = 0.0

        self.arcEndPoint = None
        self.endPoint = None
        self.finalVelocity = None
        self.arcLength = 0.0
        self.arcTime = 0.0

    def solve(self):

        solution = calcs.hitTargetAtSpeed(self.startPoint, self.speed, self.targetPoint, self.velocityOfTarget)
        if solution is None:
            raise NoSolutionException

        solutionDirection = solution.velocity / self.speed
        self.initialGuess(solutionDirection)
        iteration = 0
        while iteration < MAX_ITERATIONS:
            newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
            solution = calcs.hitTargetAtSpeed(self.arcEndPoint, self.speed, newTarget, self.velocityOfTarget)
            if solution is None:
                raise NoSolutionException
            solutionDirection = solution.velocity / self.speed
            cosError = np.dot(self.arcExitDirection, solutionDirection)
            if cosError >= MIN_ANGLE_ERROR_COS:
                self.totalTime = solution.time + self.arcTime
                self.endPoint = solution.endPoint
                self.finalVelocity = solution.velocity
                return
            self.iterateFindArc(solutionDirection)
            iteration += 1
        raise NoSolutionException

    def iterateFindArc(self, desiredFinalDirection):
        angleDiff = modAngleSigned(relativeAngleCCW(self.arcExitDirection, desiredFinalDirection))
        self.arcLength += angleDiff
        if self.arcLength < 0.0 or self.arcLength > 2.0 * math.pi:
            raise NoSolutionException

        self.arcEndPoint = self.arcCenter + calcs.rotate2d(self.fromCenter, self.arcLength)
        self.arcTime = self.arcLength * self.arcRadius / self.speed
        self.arcExitDirection = calcs.rotate2d(self.startDirection, self.arcLength)

    def initialGuess(self, desiredFinalDirection):
        self.arcLength = modAngleUnsigned(relativeAngleCCW(self.arcExitDirection, desiredFinalDirection))

        self.arcEndPoint = self.arcCenter + calcs.rotate2d(self.fromCenter, self.arcLength)
        self.arcTime = self.arcLength * self.arcRadius / self.speed
        self.arcExitDirection = calcs.rotate2d(self.startDirection, self.arcLength)
