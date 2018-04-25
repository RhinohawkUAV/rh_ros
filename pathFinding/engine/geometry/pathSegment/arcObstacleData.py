import math

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs
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
        # startSpeed = np.linalg.norm(startVelocity)
        # radius = startSpeed * startSpeed
        # diff = targetPoint - startPoint
        # toCenterDir = calcs.CCWNorm(startVelocity / startSpeed)
        # toCenter = toCenterDir * radius
        #
        # rotateSign = 1
        # if np.dot(diff, toCenterDir) < 0:
        #     toCenter *= -1
        #     rotateSign = -1
        #
        # center = startPoint + toCenter
        #
        # # TODO: Move solution to the beginning to get the correct initial direction vector
        # solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        # if solution is None:
        #     return None
        #
        # cosRotate = np.dot(startVelocity, solution.velocity) / (startSpeed * np.linalg.norm(solution.velocity))
        #
        # # Both used for display, not clear if either of these actually has to be calculated
        # arcLength = math.acos(cosRotate) * rotateSign
        # startAngle = math.atan2(-toCenter[1], -toCenter[0])
        # postArcStartPoint = center + calcs.rotate2d(-toCenter, arcLength)
        # arcingTime = rotateSign * arcLength * radius / startSpeed
        # postArcTargetPoint = targetPoint + velocityOfTarget * arcingTime
        #
        # solution = calcs.hitTargetAtSpeed(postArcStartPoint, startSpeed, postArcTargetPoint, velocityOfTarget)

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

        self.initialGuess(solution.velocity / self.speed)
        print "Initial Guess: " + str(math.degrees(self.arcLength))

        for i in range(4):
            newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
            solution = calcs.hitTargetAtSpeed(self.arcEndPoint, self.speed, newTarget, self.velocityOfTarget)
            if solution is None:
                raise NoSolutionException
            self.iterateFindArc(solution.velocity / self.speed)
            print "Guess " + str(i) + ": " + str(math.degrees(self.arcLength))

        self.totalTime = solution.time + self.arcTime
        self.endPoint = solution.endPoint
        self.finalVelocity = solution.velocity
        self.hasSolution = True

    def iterateFindArc(self, desiredFinalDirection):

        angleDiff = modAngleSigned(relativeAngleCCW(self.arcExitDirection, desiredFinalDirection))
        print str(math.degrees(math.atan2(desiredFinalDirection[1], desiredFinalDirection[0]))) + " - " + \
              str(math.degrees(math.atan2(self.arcExitDirection[1], self.arcExitDirection[0]))) + " = " + str(
            math.degrees(angleDiff))

        maxStep = math.pi / 2.0
        if angleDiff > maxStep:
            angleDiff = maxStep
        elif angleDiff < -maxStep:
            angleDiff = -maxStep

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
