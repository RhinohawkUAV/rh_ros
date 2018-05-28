from collections import Sequence

from pathSegment import PathSegment


class ObstacleData:

    def setInitialState(self, boundaryPoints, noFlyZones):
        """
        Set the initial state of the obstacle data at time=0.0.
        :param boundaryPoints:
        :param noFlyZones:
        :return:
        """
        pass

    def findPathSegment(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        # type: (float,Sequence,Sequence,Sequence,Sequence) -> PathSegment
        """
        Find legal path segments from a given starting point and velocity to the moving target, ending at finalSpeed.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.

        :param startTime: the time at which the path starts
        :param startPoint: where the path starts
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param targetPoint: the destination point
        :param velocityOfTarget: the velocity of the destination (this is usually a point on a NFZ, which may be moving)
        :return:
        """

    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        # type: (float,Sequence,Sequence) -> ([PathSegment],[PathSegment])
        """
        Find legal path segments from a given starting point and velocity to vertices of no fly zones.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.
        :param startTime: the time at which the path starts
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :return: a list of PathSegments describing routes from the start point to noFlyZone vertices.  This 1st list of
        valid path segments.  The 2nd list shows path segments that were filtered, for debugging purposes.
        """
        pass
