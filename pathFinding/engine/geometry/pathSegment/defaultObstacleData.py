import numpy as np

from engine.geometry import LineSegment, calcs
from obstacleData import ObstacleData


class DefaultObstacleData(ObstacleData):
    """
    A base class for Python implementations of ObstacleData.  Sub-classes should implement the following:
    createPathSegment() -- Create path segments from starting position to ending position
    filterImpossiblePathSegments() -- Filter out path segments which intersect the boundary or NoFlyZones.
    """

    def __init__(self, targetOffsetLength):
        # Initial position of no fly zone vertices, plus any offsetting at time=0.0.
        self.targetPoints = []

        # Velocity of the target points (corresponding to the NFZ they belong to).
        self.targetVelocities = []
        self.obstacleLines = []
        self.obstacleVelocities = []

        # A version of the target points at the current query time.  These are actually used to compute potential path
        # segments
        self.targetPointsAtTime = []

        # A version of the obstacle lines blocking potential paths to the targets at the current query time
        self.obstacleLinesAtTime = []

        # When finding paths to no fly zone vertices, this applies an "outward" offset to each vertex of this length
        self.targetOffsetLength = targetOffsetLength

    def setInitialState(self, initialPathFindingInput):
        del self.targetPoints[:]
        del self.targetVelocities[:]
        del self.obstacleLines[:]
        del self.obstacleVelocities[:]

        for noFlyZoneInput in initialPathFindingInput.noFlyZones:
            points = noFlyZoneInput.points
            nfzLines = []
            for i in range(0, len(points)):
                nfzLines.append(LineSegment(points[i - 1], points[i]))
                self.obstacleVelocities.append(np.array(noFlyZoneInput.velocity, np.double))

            for i in range(-1, len(points) - 1):
                pointNormal = (nfzLines[i].n + nfzLines[i + 1].n) / 2.0
                pointNormal /= np.linalg.norm(pointNormal)
                self.targetPoints.append(nfzLines[i].p2 + pointNormal * self.targetOffsetLength)
                self.targetVelocities.append(np.array(noFlyZoneInput.velocity, np.double))

            self.obstacleLines.extend(nfzLines)

        for i in range(len(initialPathFindingInput.boundaryPoints)):
            self.obstacleLines.append(
                LineSegment(initialPathFindingInput.boundaryPoints[i - 1], initialPathFindingInput.boundaryPoints[i]))
            self.obstacleVelocities.append(np.array((0, 0), np.double))

        # Create copy of same length corresponding to time = 0.0
        self.targetPointsAtTime = self.targetPoints[:]
        self.obstacleLinesAtTime = self.obstacleLines[:]

    def setQueryTime(self, time):
        for i in range(len(self.targetPoints)):
            self.targetPointsAtTime[i] = self.targetPoints[i] + self.targetVelocities[i] * time

        for i in range(len(self.obstacleLines)):
            offset = self.obstacleVelocities[i] * time
            self.obstacleLinesAtTime[i] = LineSegment(self.obstacleLines[i].p1 + offset,
                                                      self.obstacleLines[i].p2 + offset)

    def findPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        pathSegment = self.createPathSegment(startPoint, startVelocity, targetPoint, velocityOfTarget)

        if pathSegment is not None and self.filterPathSegment(pathSegment, self.obstacleLinesAtTime,
                                                              self.obstacleVelocities):
            return pathSegment
        else:
            return None

    def findPathSegments(self, startPoint, startVelocity):
        pathSegments = []
        for i in range(len(self.targetPoints)):
            velocityOfTarget = self.targetVelocities[i]
            targetPoint = self.targetPointsAtTime[i]
            # startPosition will typically be a NFZ vertex.  We want to eliminate search from a start position to itself.
            if not calcs.arePointsClose(startPoint, targetPoint):
                pathSegment = self.createPathSegment(startPoint, startVelocity, targetPoint, velocityOfTarget)
                if pathSegment is not None:
                    pathSegments.append(pathSegment)

        filteredPathSegments = []
        for pathSegment in pathSegments:
            if self.filterPathSegment(pathSegment, self.obstacleLinesAtTime, self.obstacleVelocities):
                filteredPathSegments.append(pathSegment)
        return filteredPathSegments

    def findPathToGoal(self, startPoint, startVelocity, goalPoint):
        pathSegment = self.createPathSegment(startPoint, startVelocity, goalPoint, np.array((0, 0), np.double))
        if pathSegment is not None and self.filterPathSegment(pathSegment, self.obstacleLinesAtTime,
                                                              self.obstacleVelocities):
            return pathSegment
        return None

    def createPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        """
        Creates a PathSegment object from the given start point and velocity, which will hit the target, which is moving
        at a given velocity.
        :param startPoint:
        :param startVelocity:
        :param targetPoint:
        :param velocityOfTarget:
        :return:
        """
        pass

    def filterPathSegment(self, pathSegment, obstacleLinesAtTime, obstacleVelocities):
        pass
