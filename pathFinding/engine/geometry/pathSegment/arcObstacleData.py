import math

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs, arc
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment

MAX_ITERATIONS = 4

# There will be difference between an arc's exit velocity and the correct velocity vector.
# This represents the allowed angular error (in radians) and corresponds to missing a target
# 1km away by 1m
MAX_ANGLE_ERROR = 0.001

MIN_ANGLE_ERROR_COS = math.cos(MAX_ANGLE_ERROR)


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
                                  arcFinder.speed, arcFinder.arc, arcFinder.arcTime)
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

        self.arc = arc.createArc(self.startPoint, self.startVelocity, acceleration, rotationDirection)
        self.totalTime = 0.0
        self.arcTime = 0.0
        self.endPoint = None
        self.finalVelocity = None

    def solve(self):
        solution = calcs.hitTargetAtSpeed(self.startPoint, self.speed, self.targetPoint, self.velocityOfTarget)
        if solution is None:
            raise NoSolutionException

        solutionDirection = solution.velocity / self.speed
        self.initialGuess(solutionDirection)
        iteration = 0
        while iteration < MAX_ITERATIONS:
            newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
            solution = calcs.hitTargetAtSpeed(self.arc.endPoint, self.speed, newTarget, self.velocityOfTarget)
            if solution is None:
                raise NoSolutionException
            solutionDirection = solution.velocity / self.speed
            cosError = np.dot(self.arc.endTangent, solutionDirection)
            if cosError >= MIN_ANGLE_ERROR_COS:
                self.totalTime = solution.time + self.arcTime
                self.endPoint = solution.endPoint
                self.finalVelocity = solution.velocity
                return
            self.iterateFindArc(solutionDirection)
            iteration += 1
        raise NoSolutionException

    def iterateFindArc(self, desiredFinalDirection):
        angleDiff = calcs.modAngleSigned(calcs.relativeAngleCCW(self.arc.endTangent, desiredFinalDirection))
        newArcLength = self.arc.length + angleDiff
        if newArcLength < 0.0 or newArcLength > 2.0 * math.pi:
            raise NoSolutionException
        self.arc.setLength(newArcLength)
        self.arcTime = self.arc.length * self.arc.radius / self.speed

    def initialGuess(self, desiredFinalDirection):
        self.arc.setLength(calcs.modAngleUnsigned(calcs.relativeAngleCCW(self.arc.endTangent, desiredFinalDirection)))
        self.arcTime = self.arc.length * self.arc.radius / self.speed
