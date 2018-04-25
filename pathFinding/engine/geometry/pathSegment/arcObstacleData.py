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

        startSpeed = np.linalg.norm(startVelocity)
        radius = startSpeed * startSpeed
        diff = targetPoint - startPoint
        toCenterDir = calcs.CCWNorm(startVelocity / startSpeed)
        toCenter = toCenterDir * radius

        rotateSign = 1
        if np.dot(diff, toCenterDir) < 0:
            toCenter *= -1
            rotateSign = -1

        center = startPoint + toCenter

        # TODO: Move solution to the beginning to get the correct initial direction vector
        solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        if solution is None:
            return None

        cosRotate = np.dot(startVelocity, solution.velocity) / (startSpeed * np.linalg.norm(solution.velocity))

        # Both used for display, not clear if either of these actually has to be calculated
        arcLength = math.acos(cosRotate) * rotateSign
        startAngle = math.atan2(-toCenter[1], -toCenter[0])
        postArcStartPoint = center + calcs.rotate2d(-toCenter, arcLength)
        arcingTime = rotateSign * arcLength * radius / startSpeed
        postArcTargetPoint = targetPoint + velocityOfTarget * arcingTime

        solution = calcs.hitTargetAtSpeed(postArcStartPoint, startSpeed, postArcTargetPoint, velocityOfTarget)
        if solution is not None:
            return ArcPathSegment(postArcStartPoint, startSpeed, solution.time + arcingTime, solution.endPoint,
                                  solution.velocity, center, radius, startAngle, arcLength, arcingTime)
        return None
