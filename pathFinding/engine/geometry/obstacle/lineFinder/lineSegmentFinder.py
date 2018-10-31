import math

from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.circularTarget import CircularTarget
from engine.geometry.obstacle.lineFinder.linePathSegment import LinePathSegment
from engine.geometry.obstacle.pathSegmentFinder import PathSegmentFinder
from engine.geometry.obstacle.vertexTarget import VertexTarget
import numpy as np

MAX_TURN_ANGLE = 60

# For calculation convenience
MAX_TURN_ANGLE_COS = math.cos(math.radians(MAX_TURN_ANGLE))


def turnIsLegal(speed, unitVelocity, velocity2):
    """
    Assumes all velocities have equal magnitude and only need their relative angle checked.
    :param velocity1:
    :param velocity2:
    :return:
    """
    cosAngle = np.dot(unitVelocity, velocity2) / speed
    return cosAngle > MAX_TURN_ANGLE_COS


class LineSegmentFinder(PathSegmentFinder):

    def __init__(self, params, vehicle):
        PathSegmentFinder.__init__(self, params, vehicle)

    def _createCircularTarget(self, center, radius, velocity):
        return CircularTarget(center, velocity, radius, self.params.nfzBufferWidth, self.params.nfzTargetOffset)   

    def _createVertexTarget(self, vertexPosition, velocity, vertexNormal, vertexAngle):
        return VertexTarget(vertexPosition, velocity, vertexNormal, vertexAngle)

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget, legalRotDirection):
        solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        if solution is not None and turnIsLegal(startSpeed, startUnitVelocity, solution.velocity):
            endPoint = solution.endPoint
            return [LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed)]
        return []

    # TODO: Merge logic into parent class
    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection):
        pathSegments = self._findStaticPathSegments(startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection)
        pathSegments.extend(self._findDynamicPathSegments(startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection))
        return pathSegments

    def _findStaticPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        pathSegments = []
        for target in self.vertexTargets:
            target.update(startTime)
            if not calcs.arePointsClose(startPoint, target.position):
                solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, target.position, target.velocity)
                if solution is not None and turnIsLegal(startSpeed, startUnitVelocity, solution.velocity):
                    endPoint = solution.endPoint                
                    pathSegment = LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed)
                    pathSegments.append(pathSegment)

        return pathSegments

    def _findDynamicPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        pathSegments = []
        for target in self.circularTargets:
            target.update(startTime)
            try:
                solutions = calcs.passTargetCircleAtSpeed(startPoint,
                                                         startSpeed,
                                                         target.position,
                                                         target.velocity,
                                                         target.radius,
                                                         target.targetRadius)
                for solution in solutions:
                    if solution is not None:
                        endPoint = solution.endPoint
                        pathSegments.append(LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed))
            except NoSolutionException:
                pass
                
        return pathSegments

