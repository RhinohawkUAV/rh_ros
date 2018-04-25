import math

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment


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

        arcFinder = ArcFinder(startPoint, startVelocity, targetPoint, velocityOfTarget)
        arcFinder.solve()

        if not arcFinder.hasSolution:
            return None

        return ArcPathSegment(arcFinder.totalTime, arcFinder.endPoint, arcFinder.finalVelocity,
                              arcFinder.speed, arcFinder.arcEndPoint, arcFinder.arcStart, arcFinder.arcLength,
                              arcFinder.arcCenter, arcFinder.arcRadius,
                              arcFinder.arcTime)


class ArcFinder:
    def __init__(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        self.startPoint = startPoint
        self.startVelocity = startVelocity
        self.speed = np.linalg.norm(startVelocity)
        self.startDirection = startVelocity / self.speed
        self.targetPoint = targetPoint
        self.velocityOfTarget = velocityOfTarget
        self.totalTime = 0.0

        self.arcEndPoint = None
        self.endPoint = None
        self.finalVelocity = None
        self.arcStart = 0.0
        self.arcLength = 0.0
        self.arcCenter = 0.0
        self.arcRadius = 0.0
        self.arcTime = 0.0
        self.hasSolution = False

    def solve(self):
        solution = calcs.hitTargetAtSpeed(self.startPoint, self.speed, self.targetPoint, self.velocityOfTarget)
        if solution is None:
            self.hasSolution = False
            return

        self.findArc(solution.velocity, 1.0)
        newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
        solution = calcs.hitTargetAtSpeed(self.arcEndPoint, self.speed, newTarget, self.velocityOfTarget)
        if solution is None:
            self.hasSolution = False
            return

        self.totalTime = solution.time + self.arcTime
        self.endPoint = solution.endPoint
        self.finalVelocity = solution.velocity
        self.hasSolution = True

    def findArc(self, finalVelocity, rotateSign):
        self.arcRadius = self.speed * self.speed
        toCenterDir = calcs.CCWNorm(self.startDirection)
        toCenter = toCenterDir * self.arcRadius * rotateSign
        self.arcCenter = self.startPoint + toCenter

        cosRotate = np.dot(self.startDirection, finalVelocity) / self.speed
        self.arcLength = math.acos(cosRotate) * rotateSign
        self.arcStart = math.atan2(-toCenter[1], -toCenter[0])
        self.arcEndPoint = self.arcCenter + calcs.rotate2d(-toCenter, self.arcLength)
        self.arcTime = rotateSign * self.arcLength * self.arcRadius / self.speed
