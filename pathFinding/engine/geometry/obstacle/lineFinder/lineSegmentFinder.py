import math

from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.circularTarget import CircularTarget
from engine.geometry.obstacle.pathSegmentFinder import PathSegmentFinder
from engine.geometry.obstacle.vertexTarget import VertexTarget
from engine.geometry.obstacle.lineFinder.linePathSegment import LinePathSegment
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

    def __init__(self, targetOffsetLength):
        self.targetOffsetLength = targetOffsetLength
        self.vertexTargets = []
        self.circularTargets = []        

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        self.circularTargets = []
        for dfnz in dynamicNoFlyZones:
            self.circularTargets.append(CircularTarget(dfnz.center, dfnz.velocity, dfnz.radius + self.targetOffsetLength))
        
    def createVertexTarget(self, point, velocity, normal, pointAngle):
        self.vertexTargets.append(VertexTarget(point, velocity, normal, pointAngle))

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        solution = calcs.hitTargetAtSpeed(startPoint, startSpeed, targetPoint, velocityOfTarget)
        if solution is not None and turnIsLegal(startSpeed, startUnitVelocity, solution.velocity):
            endPoint = solution.endPoint
            return [LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed)]
        return []

    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        pathSegments = self._findStaticPathSegments(startTime, startPoint, startSpeed, startUnitVelocity)
        pathSegments.extend(self._findDynamicPathSegments(startTime, startPoint, startSpeed, startUnitVelocity))
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
                solutions = calcs.hitTargetCircleAtSpeed(startPoint,
                                                         startSpeed,
                                                         target.position,
                                                         target.velocity,
                                                         target.radius)
                for solution in solutions:
                    if solution is not None:
                        endPoint = solution.endPoint
                        pathSegments.append(LinePathSegment(startTime, startPoint, startSpeed, solution.time, endPoint, startSpeed, solution.velocity / startSpeed))
            except NoSolutionException:
                pass
                
        return pathSegments

