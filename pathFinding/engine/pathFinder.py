from engine import waypoint
from engine.geometry.obstacle import obstacleCourse
from engine.interface.solutionWaypoint import SolutionWaypoint
from engine.vertex import UniqueVertexQueue
from engine.vertex.vertex import OriginVertex
from engine.vertex.vertexPriorityQueue import QueueEmptyException
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

        self._start = scenario.startPoint
        velocity = np.array(scenario.startVelocity, np.double)
        startSpeed = np.linalg.norm(velocity)
        unitVelocity = velocity / startSpeed        
        self._waypoints = waypoint.calcWaypoints(scenario.wayPoints, startSpeed, vehicle.acceleration)

        # No solution possible, will never put anything in the vertex queue or start any calculations.
        if startSpeed == 0.0:
            return    
        
        self._currentVertex = OriginVertex(self._waypoints[0], self._start, startSpeed, unitVelocity)

        self._pathSegments = []
        self._filteredPathSegments = []

        self._vertexQueue.push(self._currentVertex)
        self._solution = None
        self._solutionTime = float("inf")
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
            while self._currentVertex.getTimeThrough() > self._solutionTime:
                self._currentVertex = self._vertexQueue.pop()
            
            del self._pathSegments[:]
            del self._filteredPathSegments[:]
            
            if self.checkPathToGoal():
                return True
            else:
                (pathSegments, filteredPathSegments) = self._currentVertex.skirtingPathSegments(self._obstacleCourse)
                self._pathSegments.extend(pathSegments)
                self._filteredPathSegments.extend(filteredPathSegments)
                for pathSegment in self._pathSegments:
                    
                    newVertex = Vertex(self._currentVertex._waypoint,
                                       self._currentVertex._waypoint.calcHeuristic(pathSegment.endPoint,
                                                                                   pathSegment.endUnitVelocity,
                                                                                   pathSegment.endSpeed,
                                                                                   self._vehicle.acceleration),
                                       previousVertex=self._currentVertex,
                                       pathSegment=pathSegment)
        
                    self._vertexQueue.push(newVertex)
                return False
        except QueueEmptyException:
            self._isDone = True
            return True
    
    def pathToNextWaypoint(self):
        (pathSegments, filteredPathSegments) = self._currentVertex.pathSegmentsToPoint(self._obstacleCourse,
                                                                                        targetPoint=self._currentVertex._waypoint._position,
                                                                                        velocityOfTarget=self._zero)
        self._pathSegments.extend(pathSegments)
        self._filteredPathSegments.extend(filteredPathSegments)
        nextWaypoint = self._currentVertex._waypoint._nextWayPoint
        for pathSegment in self._pathSegments:
            heuristicToGoal = nextWaypoint.calcHeuristic(pathSegment.endPoint,
                                                         pathSegment.endUnitVelocity,
                                                         pathSegment.endSpeed,
                                                         self._vehicle.acceleration)
            waypointVertex = Vertex(nextWaypoint,
                                    heuristicToGoal,
                                    previousVertex=self._currentVertex,
                                    pathSegment=pathSegment)
            self._vertexQueue.push(waypointVertex)

    def checkPathToGoal(self):
        """
        Check if there is a path from self._currentVertex to the goal.  Update the best solution if this is better.
        :return:
        """
        if self._currentVertex._waypoint._nextWayPoint is not None:
            self.pathToNextWaypoint()
            return False
        
        (pathSegments, filteredPathSegments) = self._currentVertex.pathSegmentsToPoint(self._obstacleCourse,
                                                                    targetPoint=self._currentVertex._waypoint._position,
                                                                    velocityOfTarget=self._zero)
        self._pathSegments.extend(pathSegments)
        self._filteredPathSegments.extend(filteredPathSegments)
        
        for pathSegment in self._pathSegments:
            waypointVertex = Vertex(None,
                                    0.0,
                                    previousVertex=self._currentVertex,
                                    pathSegment=pathSegment)            
            if waypointVertex.getTimeThrough() < self._solutionTime:
                # TODO: Multiple solutions possible
                self._solutionTime = waypointVertex.getTimeThrough()
                self._solution = waypointVertex
                return True
        return False

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
