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
        self._params = params
        self._vehicle = vehicle
        self._obstacleCourse = obstacleCourse.createObstacleCourse(params, vehicle)
        self._obstacleCourse.setScenarioState(scenario)

        self._start = scenario.startPoint
        velocity = np.array(scenario.startVelocity, np.double)
        startSpeed = np.linalg.norm(velocity)
        unitVelocity = velocity / startSpeed        
        self._waypoints = waypoint.calcWaypoints(scenario.wayPoints, startSpeed, vehicle.acceleration)

        # Calculate bounding rectangle and use that for dimensions of the UniqueVertexQueue
        bounds = scenario.calcBounds()
        self._vertexQueue = UniqueVertexQueue(bounds[0], bounds[1], bounds[2] - bounds[0], bounds[3] - bounds[1], vehicle.maxSpeed, len(self._waypoints))

        # No solution possible, will never put anything in the vertex queue or start any calculations.
        if startSpeed == 0.0:
            return    
        
        self._currentVertex = OriginVertex(self._waypoints[0], self._start, startSpeed, unitVelocity)

        self._pathSegments = []
        self._filteredPathSegments = []

        self._vertexQueue.push(self._currentVertex)
        self._solution = None
        self._solutionTime = float("inf")
        self._solutionUpdated = False
        self._isDone = False

    def findPath(self):
        while not self.isDone():
            self.step()

    def isDone(self):
        return self._isDone
    
    @profile.accumulate("step")
    def step(self):
        try:
            del self._pathSegments[:]
            del self._filteredPathSegments[:]

            self._currentVertex = self._nextVertex()
            
            nextWaypoint = self._currentVertex.getWaypoint().getNext()
            if nextWaypoint is None:
                self._findPathsToEnd()
            else:
                (psegs, fpsegs) = self._currentVertex.pathSegmentsToWaypoint(self._obstacleCourse)
                self._addSegments(psegs, fpsegs, nextWaypoint)
                    
            (psegs, fpsegs) = self._currentVertex.skirtingPathSegments(self._obstacleCourse)
            self._addSegments(psegs, fpsegs, self._currentVertex.getWaypoint())
        except QueueEmptyException:
            self._isDone = True

    def hasSolution(self):
        return self._solution is not None

    def solutionUpdated(self):
        return self._solutionUpdated
    
    def getSolution(self):
        self._solutionUpdated = False
        pathSegments = self._getPathSegments(self._solution)
        wayPoints = self._calcSolutionWaypoints(pathSegments)
        return (wayPoints, pathSegments)
    
    def getDebugData(self):
        previousPathSegments = self._getPathSegments(self._currentVertex)
        return (previousPathSegments, self._pathSegments, self._filteredPathSegments)

    def _nextVertex(self):
        vertex = self._vertexQueue.pop()
        while vertex.getTimeThrough() > self._solutionTime:
            vertex = self._vertexQueue.pop()
        return vertex
        
    def _addSegments(self, psegs, fpsegs, waypoint):
        self._pathSegments.extend(psegs)
        self._filteredPathSegments.extend(fpsegs)
        for pathSegment in psegs:
            newVertex = Vertex(waypoint,
                               waypoint.calcHeuristic(pathSegment.endPoint,
                                                      pathSegment.endUnitVelocity,
                                                      pathSegment.endSpeed,
                                                      self._vehicle.acceleration),
                               previousVertex=self._currentVertex,
                               pathSegment=pathSegment)
            self._vertexQueue.push(newVertex)

    def _findPathsToEnd(self):
        """
        Check if there is a path from self._currentVertex to the goal.  Update the best solution if this is better.
        :return:
        """
        (psegs, fpsegs) = self._currentVertex.pathSegmentsToWaypoint(self._obstacleCourse)
        self._pathSegments.extend(psegs)
        self._filteredPathSegments.extend(fpsegs)
        
        for pathSegment in self._pathSegments:
            waypointVertex = Vertex(None,
                                    0.0,
                                    previousVertex=self._currentVertex,
                                    pathSegment=pathSegment)            
            if waypointVertex.getTimeThrough() < self._solutionTime:
                self._solutionTime = waypointVertex.getTimeThrough()
                self._solution = waypointVertex
                self._solutionUpdated = True

    def _getPathSegments(self, pathEndVertex):
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

    def _calcSolutionWaypoints(self, pathSegments):
        solutionWaypoints = []
        for pathSegment in pathSegments:
            position = pathSegment.endPoint + pathSegment.endUnitVelocity * self._params.waypointAcceptanceRadii
            solutionWaypoints.append(SolutionWaypoint(position, self._params.waypointAcceptanceRadii))
            
        return solutionWaypoints
