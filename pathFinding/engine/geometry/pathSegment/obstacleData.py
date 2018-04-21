from collections import Sequence

from pathSegment import PathSegment


class ObstacleData:
    def setInitialState(self, initialPathFindingInput):
        """
        Set the initial state of the obstacle data at time=0.0.
        :param initialPathFindingInput:
        :return:
        """
        pass

    def setQueryTime(self, time):
        """
        Does whatever internal work is necessary to update obstacle data to allow queries to be performed at
        the given time.
        :param time:
        :return:
        """
        pass

    def findPathSegment(self, startPoint, startVelocity, targetPoint, velocityOfTarget):
        # type: (Sequence,Sequence,Sequence,Sequence) -> PathSegment
        """
        Find legal path segments from a given starting point and velocity to the moving target, ending at finalSpeed.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.

        :param startPoint: where the path starts
        :param startVelocity: the initial velocity, of the vehicle at the start of the path
        :param targetPoint: the destination point
        :param velocityOfTarget: the velocity of the destination (this is usually a point on a NFZ, which may be moving)
        :return:
        """

    def findPathSegments(self, startPoint, startVelocity):
        # type: (Sequence,Sequence) -> [PathSegment]
        """
        Find legal path segments from a given starting point and velocity to vertices of no fly zones.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.
        :param startPoint:
        :param startVelocity:
        :return: a list of PathSegments describing routes from the start point to noFlyZone vertices.
        """
        pass
