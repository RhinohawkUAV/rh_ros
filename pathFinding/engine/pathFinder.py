import math
import time

from engine.geometry.obstacle import obstacleCourse
from engine.interface.solutionWaypoint import SolutionWaypoint
from engine.vertex import UniqueVertexQueue
from engine.vertex.vertex import OriginVertex
from engine.vertex.vertexPriorityQueue import QueueEmptyException
from geometry import calcs
import numpy as np
from utils import profile
from vertex import Vertex


class PathFinder:

    @profile.accumulate("setup")
    def __init__(self, params, scenario, vehicle):
        self._zero = np.array((0, 0), np.double)
        self._params = params
        self._vehicle = vehicle
        self._obstacleCourse = obstacleCourse.createObstacleCourse(params, vehicle)
        self._obstacleCourse.setScenarioState(scenario)

        # Calculate bounding rectangle and use that for dimensions of the UniqueVertexQueue
        bounds = scenario.calcBounds()
        self._vertexQueue = UniqueVertexQueue(bounds[0], bounds[1], bounds[2] - bounds[0], bounds[3] - bounds[1], vehicle.maxSpeed)

        self._start = np.array(scenario.startPoint, np.double)
        self._goal = np.array(scenario.wayPoints[0], np.double)
        velocity = np.array(scenario.startVelocity, np.double)
        startSpeed = np.linalg.norm(velocity)
        unitVelocity = velocity / startSpeed

        # No solution possible, will never put anything in the vertex queue or start any calculations.
        if startSpeed == 0.0:
            return    
        
        self._currentVertex = OriginVertex(self._start, startSpeed, unitVelocity)

        self._pathSegments = []
        self._filteredPathSegments = []
        self._solution = None
        self._bestSolutionTime = float("inf")

        self._vertexQueue.push(self._currentVertex)
        self._numQueuedVertices = 1
        self._computeTime = 0.0
        self._queueComputeTime = 0.0
        self._isDone = False

    def findPath(self):
        while not self.isDone():
            self.step()

    def isDone(self):
        return self._isDone
    
    @profile.accumulate("step")
    def step(self):
        try:
            self._currentVertex = self._vertexQueue.pop()
            while self._currentVertex.getTimeThrough() > self._bestSolutionTime:
                self._currentVertex = self._vertexQueue.pop()
            
            if self.checkPathToGoal():
                return True
            else:
                (self._pathSegments, self._filteredPathSegments) = self._currentVertex.skirtingPathSegments(self._obstacleCourse)

                for pathSegment in self._pathSegments:
                    
                    newVertex = Vertex(self.heuristic(pathSegment.endPoint,
                                                      pathSegment.endSpeed,
                                                      pathSegment.endUnitVelocity),
                                       previousVertex=self._currentVertex,
                                       pathSegment=pathSegment)
        
                    self._vertexQueue.push(newVertex)
                return False
        except QueueEmptyException:
            self._isDone = True
            return True
            
    def checkPathToGoal(self):
        """
        Check if there is a path from self._currentVertex to the goal.  Update the best solution if this is better.
        :return:
        """
        (pathSegments, x) = self._currentVertex.pathSegmentsToPoint(self._obstacleCourse,
                                                                    targetPoint=self._goal,
                                                                    velocityOfTarget=self._zero)
        if len(pathSegments) == 0:
            return False
        for pathSegment in pathSegments:
            timeToGoal = self._currentVertex.getTimeTo() + pathSegment.elapsedTime
            if timeToGoal < self._bestSolutionTime:
                self._solution = Vertex(0.0,
                                        previousVertex=self._currentVertex,
                                        pathSegment=pathSegment)

                self._bestSolutionTime = timeToGoal
        return True
        
    def heuristic(self, point, speed, unitVelocity):
        # TODO: Heuristic should be based on obstacle data and in the case of arcs should account for the heading at a given point.
        # This could use the, possibly filtered path, from the current vertex, to the goal
        
        # Approximate time is based on time to turn plus straightline distance.  
        # This can give an over estimate making this method no longer admissible.  Given how open ended the search is, this is fine.
        (direction, distance) = calcs.unitAndLength(self._goal - point)
        
        # TODO: This should not be necessary and doesn't account for -1.0.  This seems to be some kind of result of float64
        turnAngleCos = min(1.0, np.dot(unitVelocity, direction))
        
        turnAngle = math.acos(turnAngleCos)
        turnTime = turnAngle * speed / self._vehicle.acceleration
        straightTime = distance / speed
        return turnTime + straightTime
#         return calcs.calcTravelTime(point, self._goal, speed)

    def hasSolution(self):
        return self._solution is not None
    
    def getSolution(self):
        pathSegments = self.getPathSegments(self._solution)
        wayPoints = self.calcSolutionWaypoints(pathSegments)

        return (wayPoints, pathSegments)
    
    def getDebugData(self):
        previousPathSegments = self.getPathSegments(self._currentVertex)
        return (previousPathSegments, self._pathSegments, self._filteredPathSegments)
        
    def getPathSegments(self, pathEndVertex):
        pathSegments = []
        currentVertex = pathEndVertex
        previousVertex = currentVertex.getPreviousVertex()

        while not previousVertex is None:
            pathSegments.append(currentVertex.pathSegment)
            currentVertex = previousVertex
            previousVertex = currentVertex.getPreviousVertex()
        
        # We traced the path backwards, so reverse
        pathSegments.reverse()

        return pathSegments

    def calcSolutionWaypoints(self, pathSegments):
        solutionWaypoints = []
        for pathSegment in pathSegments:
            position = pathSegment.endPoint + pathSegment.endUnitVelocity * self._params.waypointAcceptanceRadii
            solutionWaypoints.append(SolutionWaypoint(position, self._params.waypointAcceptanceRadii))
            
        return solutionWaypoints
