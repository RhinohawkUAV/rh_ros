import numpy as np

from engine.geometry import LineSegment, calcs
from obstacleData import ObstacleData


class DefaultObstacleData(ObstacleData):
    """
    A base class for Python implementations of ObstacleData.  Sub-classes should implement the following:
    createPathSegment() -- Create path segments from starting position to ending position
    filterImpossiblePathSegments() -- Filter out path segments which intersect the boundary or NoFlyZones.
    """

    def __init__(self, boundaryPoints, noFlyZones):
        ObstacleData.__init__(self, boundaryPoints, noFlyZones)

        self.obstacleLines = []
        self.obstacleVelocities = []

        for noFlyZone in noFlyZones:
            for line in noFlyZone._lines:
                self.obstacleLines.append(line)
                self.obstacleVelocities.append(np.array(noFlyZone._velocity, np.double))

        for i in range(len(boundaryPoints)):
            self.obstacleLines.append(LineSegment(boundaryPoints[i - 1], boundaryPoints[i]))
            self.obstacleVelocities.append(np.array((0, 0), np.double))

        self.targetPointsAtTime = self.targetPoints[:]
        self.obstacleLinesAtTime = self.obstacleLines[:]

    def setQueryTime(self, time):
        for i in range(len(self.targetPoints)):
            self.targetPointsAtTime[i] = self.targetPoints[i] + self.targetVelocities[i] * time

        for i in range(len(self.obstacleLines)):
            offset = self.obstacleVelocities[i] * time
            self.obstacleLinesAtTime[i] = LineSegment(self.obstacleLines[i].p1 + offset,
                                                      self.obstacleLines[i].p2 + offset)

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

    def filterPathSegment(self, pathSegments, obstacleLinesAtTime, obstacleVelocities):
        pass
