import math

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs, arc
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment


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

        self.initialGuess(solution.velocity / self.speed)
        print "Initial Guess: " + str(math.degrees(self.arc.length))

        for i in range(4):
            newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
            solution = calcs.hitTargetAtSpeed(self.arc.endPoint, self.speed, newTarget, self.velocityOfTarget)
            if solution is None:
                raise NoSolutionException
            self.iterateFindArc(solution.velocity / self.speed)
            print "Guess " + str(i) + ": " + str(math.degrees(self.arc.length))

        self.totalTime = solution.time + self.arcTime
        self.endPoint = solution.endPoint
        self.finalVelocity = solution.velocity

    def iterateFindArc(self, desiredFinalDirection):

        angleDiff = modAngleSigned(relativeAngleCCW(self.arc.endTangent, desiredFinalDirection))
        print str(math.degrees(math.atan2(desiredFinalDirection[1], desiredFinalDirection[0]))) + " - " + \
              str(math.degrees(math.atan2(self.arc.endTangent[1], self.arc.endTangent[0]))) + " = " + str(
            math.degrees(angleDiff))

        maxStep = math.pi / 2.0
        if angleDiff > maxStep:
            angleDiff = maxStep
        elif angleDiff < -maxStep:
            angleDiff = -maxStep

        newArcLength = self.arc.length + angleDiff

        if newArcLength < 0.0 or newArcLength > 2.0 * math.pi:
            raise NoSolutionException
        self.arc.setLength(newArcLength)
        self.arcTime = self.arc.length * self.arc.radius / self.speed

    def initialGuess(self, desiredFinalDirection):
        self.arc.setLength(modAngleUnsigned(relativeAngleCCW(self.arc.endTangent, desiredFinalDirection)))
        self.arcTime = self.arc.length * self.arc.radius / self.speed
