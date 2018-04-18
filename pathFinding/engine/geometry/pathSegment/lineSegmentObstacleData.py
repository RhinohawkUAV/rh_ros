import numpy as np

from constants import MAX_TURN_ANGLE_COS
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs
from linePathSegment import LinePathSegment


class LineSegmentObstacleData(DefaultObstacleData):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, constantSpeed, boundaryPoints, noFlyZones):
        DefaultObstacleData.__init__(self, boundaryPoints, noFlyZones)
        self.constantSpeed = constantSpeed

        # nfzPoints = 0
        # for noFlyZone in noFlyZones:
        #     nfzPoints += len(noFlyZone._points)
        #
        # numTargets = nfzPoints
        # numLines = nfzPoints + len(boundaryPoints)

        # self.targetPoints = np.zeros(numTargets,np.double)
        # self.targetVelocities = np.zeros(numTargets,np.double)
        # self.lineObstacles = np.zeros(numLines,np.double)
        # self.lineObstacles = np.zeros(numLines,np.double)

    def createPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        solution = calcs.hitTargetAtSpeed(startPoint, self.constantSpeed, targetPoint, velocityOfTarget)
        if solution is not None and turnIsLegal(startVelocity, solution.velocity, self.constantSpeed):
            return LinePathSegment(startPoint, self.constantSpeed, solution.time, solution.destination,
                                   solution.velocity)
        return None

    def filterPathSegment(self, linePathSegment, obstacleLines, obstacleVelocities):
        for i in range(len(obstacleLines)):
            obstacleLine = obstacleLines[i]
            obstacleLineVelocity = obstacleVelocities[i]
            if linePathSegment.doesLineIntersect(obstacleLine, obstacleLineVelocity):
                return False
        return True


def turnIsLegal(velocity1, velocity2, speed):
    """
    Assumes all velocities have equal magnitude and only need their relative angle checked.
    :param velocity1:
    :param velocity2:
    :return:
    """
    if velocity1[0] == 0.0 and velocity1[1] == 0.0:
        return True
    cosAngle = np.dot(velocity1, velocity2) / (speed * speed)
    return cosAngle > MAX_TURN_ANGLE_COS
