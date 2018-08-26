from _collections import deque
import math
from numpy.random.mtrand import np

from engine.geometry import calcs


class PathFindWaypoint:
    """
    Tracks information about each waypoint in a path-finding problem.
    """

    def __init__(self, position):
        self._position = position
        self._solutionVertex = None
        self._solutionTime = float("inf")

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
            1. Time to turn and face the waypoint
            2. Time to reach the waypoint, in a straight line after the turn (does not account the positional offset the turn introduces)
            3. Once the waypoint is reached, the time to turn to face the next waypoint
            4. Finally, once all this has been calculated the, precomputed, remaining cost is added
        """
        (directionAtWaypoint, distance) = calcs.unitAndLength(self._position - startPoint)
        time = distance / startSpeed + turnTime(startDirection, directionAtWaypoint, startSpeed, acceleration)
        time += self.calcHeuristicFromWaypoint(directionAtWaypoint, startSpeed, acceleration)
        return time 
    
    def calcHeuristicFromWaypoint(self, startDirection, startSpeed, acceleration):
        """
        Optimistic estimate for time to reach the end, starting at this waypoint with a given heading and speed.
        Cost is based on:
            1. The time to turn to face the next waypoint
            2. The, precomputed, time to the end
        """
        time = self._heuristicToEnd
        # Add time, after reaching this waypoint to turn towards the next waypoint
        if self._nextWayPointDirection is not None:
            time += turnTime(startDirection, self._nextWayPointDirection, startSpeed, acceleration)
        return time 

    def updateSolution(self, timeToWaypoint, direction, speed, acceleration):
        time = timeToWaypoint + self.calcHeuristicFromWaypoint(direction, speed, acceleration)
        if time < self._solutionTime:
            self._solutionTime = time
            return True
        return False


# TODO: Move to obstacleCourse
def turnTime(direction, desiredDirection, speed, acceleration):
    # TODO: min should not be necessary (float64)
    turnAngleCos = min(1.0, np.dot(direction, desiredDirection))
    turnAngle = math.acos(turnAngleCos)
    return turnAngle * speed / acceleration


def calcWaypointGoals(wayPoints, speed, acceleration):
    queue = deque(wayPoints)
    pathFindWaypoints = []
    nextWayPoint = PathFindWaypoint(queue.pop())
    pathFindWaypoints.append(nextWayPoint)
    
    while len(queue) > 0:
        wayPoint = PathFindWaypoint(queue.pop())
        wayPoint.setNextWaypoint(nextWayPoint, speed, acceleration)
        pathFindWaypoints.append(wayPoint)
        nextWayPoint = wayPoint
    
    pathFindWaypoints.reverse()
    return pathFindWaypoints      
