import math
from numpy.random.mtrand import np

from engine.geometry import calcs


class Waypoint:
    """
    Tracks information about each waypoint in a path-finding problem.
    """

    def __init__(self, index, position):
        self._index = index
        self._position = position

        self._nextWayPoint = None
        # The direction which points from this waypoint directly to the next one.
        self._nextWayPointDirection = None
        
        # The cost, to get to the last waypoint, if already facing directly towards the next waypoint.
        self._heuristicToEnd = 0.0

    def setNextWaypoint(self, nextWayPoint, speed, acceleration):
        self._nextWayPoint = nextWayPoint
        (self._nextWayPointDirection, distance) = calcs.unitAndLength(nextWayPoint._position - self._position)
        self._heuristicToEnd = nextWayPoint.calcHeuristic(self._position, self._nextWayPointDirection, speed, acceleration)

    def calcHeuristic(self, startPoint, startDirection, startSpeed, acceleration):
        """
        Optimistic estimate for time to reach the end waypoint given starting conditions.  
        Cost is based on:
            1. The time to turn to face the waypoint
            2. Time to reach the waypoint, in a straight line after the turn (does not account the positional offset the turn introduces)
            If this is not the last waypoint:
                3. The, precomputed, time from waypoint to the end
                4. Once the waypoint is reached, the time to turn to face the next waypoint (if there is another waypoint)
        """
        (directionAtWaypoint, distance) = calcs.unitAndLength(self._position - startPoint)
        time = turnTime(startDirection, directionAtWaypoint, startSpeed, acceleration) + \
               distance / startSpeed
               
        if self._nextWayPointDirection is not None:
            time += self._heuristicToEnd + \
                    turnTime(startDirection, self._nextWayPointDirection, startSpeed, acceleration)
        
        return time 

    def getIndex(self):
        return self._index

    def getNext(self):
        return self._nextWayPoint
    
    def isFinal(self):
        return self._nextWayPoint is None


# TODO: Move to obstacleCourse
def turnTime(direction, desiredDirection, speed, acceleration):
    # TODO: min should not be necessary (float64)
    turnAngleCos = min(1.0, np.dot(direction, desiredDirection))
    turnAngle = math.acos(turnAngleCos)
    return turnAngle * speed / acceleration


def calcWaypoints(positions, traversalSpeed, acceleration):
    index = 0
    pathFindWaypoints = []
    for index in range(len(positions)):
        pathFindWaypoints.append(Waypoint(index, positions[index]))
    
    for index in range(len(pathFindWaypoints) - 1, 0, -1):
        pathFindWaypoints[index - 1].setNextWaypoint(pathFindWaypoints[index], traversalSpeed, acceleration)

    return pathFindWaypoints
