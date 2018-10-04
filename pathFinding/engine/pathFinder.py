import sys

from engine import waypoint
from engine.geometry.obstacle import obstacleCourse
from engine.interface import outputPath
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
        self._obstacleCourse = obstacleCourse.createObstacleCourse(params, vehicle, scenario)

        self._start = scenario.startPoint
        velocity = np.array(scenario.startVelocity, np.double)
        startSpeed = np.linalg.norm(velocity)
        unitVelocity = velocity / startSpeed        
        self._waypoints = waypoint.calcWaypoints(scenario.wayPoints, startSpeed, vehicle.acceleration)

        # Calculate bounding rectangle and use that for dimensions of the UniqueVertexQueue
        bounds = scenario.calcBounds()
        self._vertexQueue = UniqueVertexQueue(bounds[0], bounds[1], bounds[2] - bounds[0], bounds[3] - bounds[1], vehicle.maxSpeed, len(self._waypoints))

        # TODO: Eject starting conditions which are out of bounds (position outside boundaries or initial speed too high)
        # No solution possible, will never put anything in the vertex queue or start any calculations.
        if startSpeed == 0.0:
            return
        
        self._currentVertex = OriginVertex(self._waypoints[0], self._start, startSpeed, unitVelocity)

        self._pathSegments = []
        self._filteredPathSegments = []

        self._vertexQueue.push(self._currentVertex)
        
        # The best initial solution is the current vertex
        self._bestPathVertex = self._currentVertex
        
        # The best completed solution time found so far.  Any vertex whose admissible estimate is worse, is rejected immediately.
        self._pruningTime = float("inf")
        
    @profile.accumulate("step")
    def step(self):
        try:
            del self._pathSegments[:]
            del self._filteredPathSegments[:]

            self._currentVertex = self._nextVertex()

            (psegs, fpsegs) = self._currentVertex.pathSegmentsToWaypoint(self._obstacleCourse)
            if self._currentVertex.getNextWaypoint().isFinal():
                self._processSegmentsToGoal(psegs, fpsegs)
            else:
                self._processSegments(psegs, fpsegs, self._currentVertex.getNextWaypoint().getNext())
                    
            (psegs, fpsegs) = self._currentVertex.skirtingPathSegments(self._obstacleCourse)
            self._processSegments(psegs, fpsegs, self._currentVertex.getNextWaypoint())
            
            return True
        except QueueEmptyException:
            return False

    def getBestPath(self):
        """
        Generates a BestPath object representing the best path found so far.  The path may be
        incomplete, in which case it represents the solution which appears to be the best.
        """
        if self._bestPathVertex.getNextWaypoint() is None:
            numWaypointsCompleted = len(self._waypoints)
            quality = 2
            if self._vertexQueue.isEmpty():
                quality += 1
        else:
            numWaypointsCompleted = self._bestPathVertex.getNextWaypoint().getIndex()
            quality = 1
            if self._vertexQueue.isEmpty():
                quality -= 1
            
        return outputPath.generatePath(self._bestPathVertex, self._params.waypointAcceptanceRadii, quality, numWaypointsCompleted)
       
    def getDebugData(self):
        """
        Generates several useful pieces of debugging data:
        (vertexSegments,newSegments,filteredSegments)
        vertexSegments: The segments upto and through the current vertex
        newSegments: The newly found segments
        filteredSegments: Additional new segments which were considered and rejected 
        """
        return (self._currentVertex.generatePathSegments(), self._pathSegments, self._filteredPathSegments)

    def _nextVertex(self):
        vertex = self._vertexQueue.pop()
        while vertex.getTimeThroughAdmissible() > self._pruningTime:
            vertex = self._vertexQueue.pop()
        return vertex

    def _processSegments(self, psegs, fpsegs, nextWaypoint):
        self._pathSegments.extend(psegs)
        self._filteredPathSegments.extend(fpsegs)
        for pathSegment in psegs:
            heuristic = nextWaypoint.calcHeuristic(pathSegment.endPoint,
                                                      pathSegment.endUnitVelocity,
                                                      pathSegment.endSpeed,
                                                      self._vehicle.acceleration)
            newVertex = Vertex(nextWaypoint,
                               heuristic * self._params.vertexHeuristicMultiplier,
                               heuristic,
                               previousVertex=self._currentVertex,
                               pathSegment=pathSegment)
            
            # Check if this vertex is the best one found so far
            self._updateSolution(newVertex)
            self._vertexQueue.push(newVertex)

    def _processSegmentsToGoal(self, psegs, fpsegs):
        self._pathSegments.extend(psegs)
        self._filteredPathSegments.extend(fpsegs)
        for pathSegment in psegs:
            waypointVertex = Vertex(None,
                                    0.0,
                                    0.0,
                                    previousVertex=self._currentVertex,
                                    pathSegment=pathSegment)
            
            self._updateSolution(waypointVertex)

    def _updateSolution(self, vertex):
        if self._checkImprovedSolution(vertex):
            self._bestPathVertex = vertex
            
            # Only time for completed solutions is used to update pruning time
            if self._bestPathVertex.isSolution():
                self._pruningTime = self._bestPathVertex.getTimeThroughAdmissible()

    def _checkImprovedSolution(self, vertex):
        """
        Check if the given vertex is better than the solution vertex.
        """
        if self._bestPathVertex.isSolution():
            solWayPoint = sys.maxint
        else:
            solWayPoint = self._bestPathVertex.getNextWaypoint().getIndex()

        if vertex.isSolution():
            vxWayPoint = sys.maxint
        else:
            vxWayPoint = vertex.getNextWaypoint().getIndex()

        # If waypoints are the same (possibly both at final waypoint) then we compare priority which is a better indicator than admissible.
        if solWayPoint == vxWayPoint:
            return self._bestPathVertex.getTimeThroughHeuristic() > vertex.getTimeThroughHeuristic()

        return solWayPoint < vxWayPoint
        
