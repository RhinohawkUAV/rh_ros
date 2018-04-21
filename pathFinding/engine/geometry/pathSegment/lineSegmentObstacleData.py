import numpy as np

from constants import MAX_TURN_ANGLE_COS, NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs
from linePathSegment import LinePathSegment


class LineSegmentObstacleData(DefaultObstacleData):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, targetOffsetLength=NO_FLY_ZONE_POINT_OFFSET):
        DefaultObstacleData.__init__(self, targetOffsetLength)

    def createPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        startSpeed = np.linalg.norm(startVelocity)
        solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        if solution is not None and turnIsLegal(startVelocity, solution.velocity):
            # TODO: convert remaining math to nparray
            endPoint = np.array(solution.destination, np.double)
            return LinePathSegment(startPoint, startSpeed, solution.time, endPoint, solution.velocity)
        return None

    def filterPathSegment(self, linePathSegment, obstacleLines, obstacleVelocities):
        for i in range(len(obstacleLines)):
            obstacleLine = obstacleLines[i]
            obstacleLineVelocity = obstacleVelocities[i]
            if linePathSegment.doesLineIntersect(obstacleLine, obstacleLineVelocity):
                return False
        return True


def turnIsLegal(velocity1, velocity2):
    """
    Assumes all velocities have equal magnitude and only need their relative angle checked.
    :param velocity1:
    :param velocity2:
    :return:
    """
    if velocity1[0] == 0.0 and velocity1[1] == 0.0:
        return True
    cosAngle = np.dot(velocity1, velocity2) / (np.linalg.norm(velocity1) * np.linalg.norm(velocity2))
    return cosAngle > MAX_TURN_ANGLE_COS
