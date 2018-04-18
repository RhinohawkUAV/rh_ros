from collections import Sequence

import numpy as np

from pathSegment import PathSegment


class ObstacleData:
    def __init__(self, boundaryPoints, noFlyZones):

        self.targetPoints = []
        self.targetVelocities = []

        pointIndex = 0
        for noFlyZone in noFlyZones:
            for point in noFlyZone._offsetPoints:
                self.targetPoints.append(np.array(point, np.double))
                self.targetVelocities.append(np.array(noFlyZone._velocity, np.double))
                pointIndex += 1

    def setQueryTime(self, time):
        """
        Does whatever internal work is necessary to update obstacle data to allow queries to be performed at
        the given time.
        :param time:
        :return:
        """
        pass

    def findPathSegments(self, startPoint, startVelocity):
        # type: (Sequence,Sequence) -> [PathSegment]
        """
        Find legal path segments from a given starting point and velocity to vertices of no fly zones.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.
        This should suggest routes at different speeds if possible.
        :param startPoint:
        :param startVelocity:
        :return: a list of PathSegments describing routes from the start point to noFlyZone vertices.  This can suggest
        multiple routes, at different speeds, to a given vertex.
        """
        pass

    def findPathToGoal(self, startPoint, startVelocity, goalPoint):
        # type: (Sequence,Sequence) -> [PathSegment]
        """
        Find the fastest legal path segment from a given starting point and velocity to the goal, which is assumed to
        be stationary.
        This takes into account the time at which this query is made, which will affect the position of NFZs.
        Returns None if no route is possible.
        :param startPoint:
        :param startVelocity:
        :param goalPoint:
        :return: a single PathSegment or None
        """
        pass
