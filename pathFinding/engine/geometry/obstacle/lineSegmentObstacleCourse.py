import math

from engine.geometry import calcs
from engine.geometry.obstacle.defaultObstacleCourse import DefaultObstacleCourse
from engine.geometry.pathSegment.linePathSegment import LinePathSegment
import numpy as np

MAX_TURN_ANGLE = 60

# For calculation convenience
MAX_TURN_ANGLE_COS = math.cos(math.radians(MAX_TURN_ANGLE))


class LineSegmentObstacleCourse(DefaultObstacleCourse):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, targetOffsetLength):
        DefaultObstacleCourse.__init__(self, targetOffsetLength)

    def createPathSegmentToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        if solution is not None and turnIsLegal(startSpeed, startUnitVelocity, solution.velocity):
            endPoint = solution.endPoint
            
            return LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed)
        return None


def turnIsLegal(speed, unitVelocity, velocity2):
    """
    Assumes all velocities have equal magnitude and only need their relative angle checked.
    :param velocity1:
    :param velocity2:
    :return:
    """
    cosAngle = np.dot(unitVelocity, velocity2) / speed
    return cosAngle > MAX_TURN_ANGLE_COS
