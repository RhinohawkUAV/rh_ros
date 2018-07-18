import math

from engine.geometry import calcs
from engine.geometry.obstacle.obstacleCourse import ObstacleCourse 
from engine.geometry.pathSegment.defaultPathSegment import DefaultPathSegment
from engine.geometry.pathSegment.obstacleLineSegment import ObstacleLineSegment
from engine.interface import dynamicNoFlyZone
import numpy as np
from utils import profile


class DefaultObstacleCourse(ObstacleCourse):
    """
    A base class for Python implementations of ObstacleCourse.  Sub-classes should implement the following:
    createPathSegment() -- Create path segments from starting position to ending position
    filterImpossiblePathSegments() -- Filter out path segments which intersect the boundary or NoFlyZones.
    """

    def __init__(self, targetOffsetLength):
        # Initial position of no fly zone vertices, plus any offsetting at time=0.0.
        self.targetPoints = []

        # Velocity of the target points (corresponding to the NFZ they belong to).
        self.targetVelocities = []
        self.targetPointNormals = []
        self.targetCosLimits = []

        self.obstacleLines = []

        # When finding paths to no fly zone vertices, this applies an "outward" offset to each vertex of this length
        self.targetOffsetLength = targetOffsetLength

    def setInitialState(self, boundaryPoints, noFlyZones):
        del self.targetPoints[:]
        del self.targetPointNormals[:]
        del self.targetCosLimits[:]
        del self.targetVelocities[:]
        del self.obstacleLines[:]

        for noFlyZoneInput in noFlyZones:
            points = noFlyZoneInput.points
            nfzLines = []
            for i in range(0, len(points)):
                nfzLines.append(
                    ObstacleLineSegment(points[i - 1], points[i], np.array(noFlyZoneInput.velocity, np.double)))

            for i in range(-1, len(points) - 1):
                pointNormal = (nfzLines[i].n + nfzLines[i + 1].n) / 2.0
                pointNormal /= np.linalg.norm(pointNormal)
                # Angle between adjacent edges should mostly be large (>180) given winding direction
                pointAngle = calcs.calcEdgeAngle(points[i - 1], points[i], points[i + 1])
                if pointAngle > math.pi:
                    cosLimit = math.cos(pointAngle / 2.0)
                else:
                    cosLimit = 1

                self.targetPointNormals.append(pointNormal)
                self.targetCosLimits.append(cosLimit)
                self.targetPoints.append(nfzLines[i].p2 + pointNormal * self.targetOffsetLength)
                self.targetVelocities.append(np.array(noFlyZoneInput.velocity, np.double))

            self.obstacleLines.extend(nfzLines)

        for i in range(len(boundaryPoints)):
            self.obstacleLines.append(
                ObstacleLineSegment(boundaryPoints[i - 1],
                                    boundaryPoints[i], np.array((0, 0), np.double)))

        # Create copy of same length corresponding to time = 0.0
        self.targetPointsAtTime = self.targetPoints[:]
        self.obstacleLinesAtTime = self.obstacleLines[:]

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        self.dynamicNoFlyZones = dynamicNoFlyZones

    def createPathSegmentToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        """
        Creates a PathSegment object from the given start point and velocity, which will hit the target, which is moving
        at a given velocity.
        :param startTime: absolute time at start of segment
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param targetPoint:
        :param velocityOfTarget:
        :return:
        """
        pass

    def createPathSegmentsToDynamicNoFlyZone(self, startTime, startPoint, startSpeed, startUnitVelocity, dynamicNoFlyZone):
        """
        Creates a list of PathSegment objects from the given start point and velocity, which will skirt the edge of a dynamic no fly zone.
        at a given velocity.
        :param startTime: absolute time at start of segment
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param dynamicNoFlyZone:
        :return:
        """
        pass

    def findPathSegmentToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        # type: (float,Sequence,Sequence,Sequence,Sequence) -> PathSegment or None
        pathSegment = self.createPathSegmentToPoint(startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget)

        if pathSegment is not None and self._filterPathSegment(pathSegment, self.obstacleLines):
            return pathSegment
        else:
            return None

    def findPathSegmentsToDynamicNoFlyZone(self, startTime, startPoint, startSpeed, startUnitVelocity, dynamicNoFlyZone):
        pathSegments = self.createPathSegmentsToDynamicNoFlyZone(startTime, startPoint, startSpeed, startUnitVelocity, dynamicNoFlyZone)
        filteredPathSegments = []
        for pathSegment in pathSegments:
            if pathSegment is not None and self._filterPathSegment(pathSegment, self.obstacleLines):
                filteredPathSegments.append(pathSegment)
        return filteredPathSegments

    @profile.accumulate("Find Path Segments")
    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        staticPathSegments = self._findStaticPathSegments(startTime, startPoint, startSpeed, startUnitVelocity)
        dynamicPathSegments = self._findDynamicPathSegments(startTime, startPoint, startSpeed, startUnitVelocity)
        
        unfilteredPathSegments = []
        unfilteredPathSegments.extend(staticPathSegments)
        unfilteredPathSegments.extend(dynamicPathSegments)
        return self._filterPathSegments(unfilteredPathSegments, self.obstacleLinesAtTime)
        
    def _findStaticPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        pathSegments = []
        for i in range(len(self.targetPoints)):
            velocityOfTarget = self.targetVelocities[i]
            targetPoint = self.targetPoints[i] + self.targetVelocities[i] * startTime

            # TODO: Use a target indexing scheme
            # startPosition will typically be a NFZ vertex.  We want to eliminate search from a start position to itself.
            if not calcs.arePointsClose(startPoint, targetPoint):
                pathSegment = self.createPathSegmentToPoint(startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget)
                if pathSegment is not None:
                    # Filter path segment based on incoming angle and known geometry around point.
                    pointNormal = self.targetPointNormals[i]
                    cosLimit = self.targetCosLimits[i]
                    relativeVelocity = pathSegment.endSpeed * pathSegment.endUnitVelocity - velocityOfTarget
                    relativeVelocity /= np.linalg.norm((relativeVelocity))
                    if np.dot(relativeVelocity, pointNormal) >= cosLimit:
                        pathSegments.append(pathSegment)
        return pathSegments

    # TODO: Use the "buffer" constant to increase circle radius when skirting dnfzs
    def _findDynamicPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        
        pathSegments = []
        return pathSegments

    # TODO: Filtering should include dnfzs at some point
    def _filterPathSegment(self, pathSegment, obstacleLines):
        for i in range(len(obstacleLines)):
            obstacleLine = obstacleLines[i]
            if pathSegment.intersectsObstacleLine(obstacleLine):
                return False
        return True

    @profile.accumulate("Collision Detection")
    def _filterPathSegments(self, pathSegments, obstacleLines):
        unfilteredPathSegments = []
        filteredPathSegments = []
        for pathSegment in pathSegments:
            if self._filterPathSegment(pathSegment, obstacleLines):
                unfilteredPathSegments.append(pathSegment)
            else:
                filteredPathSegments.append(pathSegment)
        return (unfilteredPathSegments, filteredPathSegments)
