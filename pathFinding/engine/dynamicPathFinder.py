import time

from constants import NO_FLY_ZONE_POINT_OFFSET
from engine.geometry.pathSegment.arcObstacleData import ArcObstacleData
from engine.vertex import UniqueVertexQueue
from geometry import calcs
import numpy as np
from vertex import Vertex


class DynamicPathFinder:

    def __init__(self, scenario, vehicle):

        self._obstacleData = ArcObstacleData(vehicle.acceleration)
        # self._obstacleData = LineSegmentObstacleData(NO_FLY_ZONE_POINT_OFFSET)
        self._obstacleData.setInitialState(scenario.boundaryPoints, scenario.noFlyZones)

        # Calculate bounding rectangle and use that for dimensions of the UniqueVertexQueue
        bounds = scenario.calcBounds()
        self._vertexQueue = UniqueVertexQueue(bounds[0], bounds[1], bounds[2], bounds[3], vehicle.maxSpeed)

        self._start = np.array(scenario.startPoint, np.double)
        self._goal = np.array(scenario.wayPoints[0], np.double)
        velocity = np.array(scenario.startVelocity, np.double)
        self._currentVertex = Vertex(position=self._start,
                                     velocity=velocity,
                                     timeToVertex=0.0,
                                     estimatedTimeThroughVertex=self.heuristic(self._start, velocity))

        self._pathSegments = []
        self._filteredPathSegments = []
        self._solution = None
        self._bestSolutionTime = float("inf")

        self._vertexQueue.push(self._currentVertex)
        self._numQueuedVertices = 1
        self._computeTime = 0.0
        self._findPathsTime = 0.0
        self._queueComputeTime = 0.0

    def findPath(self):
        while not self.isDone():
            self.step()

    def isDone(self):
        return self._vertexQueue.isEmpty()

    def step(self):
        self._computeTime -= time.time()
        self._currentVertex = self._vertexQueue.pop()
        if self._currentVertex.estimatedTimeThroughVertex > self._bestSolutionTime:
            self._computeTime += time.time()
            return False

        self.checkPathToGoal()
        self._findPathsTime -= time.time()
        (self._pathSegments, self._filteredPathSegments) = self._obstacleData.findPathSegments(
            startTime=self._currentVertex.timeToVertex,
            startPoint=self._currentVertex.position,
            startVelocity=self._currentVertex.velocity)
        self._findPathsTime += time.time()
        for pathSegment in self._pathSegments:
            timeToVertex = self._currentVertex.timeToVertex + pathSegment.time

            newVertex = Vertex(position=pathSegment.endPoint,
                               velocity=pathSegment.endVelocity,
                               timeToVertex=timeToVertex,
                               estimatedTimeThroughVertex=timeToVertex + self.heuristic(pathSegment.endPoint,
                                                                                        pathSegment.endVelocity),
                               previousVertex=self._currentVertex,
                               pathSegment=pathSegment)

            self._queueComputeTime -= time.time()
            self._vertexQueue.push(newVertex)
            self._queueComputeTime += time.time()

        self._computeTime += time.time()
        return True

    def checkPathToGoal(self):
        """
        Check if there is a path from self._currentVertex to the goal.  Update the best solution if this is better.
        :return:
        """
        self._findPathsTime -= time.time()
        pathSegment = self._obstacleData.findPathSegment(startTime=self._currentVertex.timeToVertex,
                                                         startPoint=self._currentVertex.position,
                                                         startVelocity=self._currentVertex.velocity,
                                                         targetPoint=self._goal,
                                                         velocityOfTarget=np.array((0, 0), np.double))
        if pathSegment is not None:
            timeToGoal = self._currentVertex.timeToVertex + pathSegment.time
            self._findPathsTime += time.time()
            if timeToGoal < self._bestSolutionTime:
                self._solution = Vertex(position=self._goal,
                                        velocity=pathSegment.endVelocity,
                                        timeToVertex=timeToGoal,
                                        estimatedTimeThroughVertex=timeToGoal,  # timeToVertex + 0
                                        previousVertex=self._currentVertex,
                                        pathSegment=pathSegment)

                self._bestSolutionTime = timeToGoal
        else:
            self._findPathsTime += time.time()

    def heuristic(self, point, velocity):
        return calcs.calcTravelTime(point, self._goal, np.linalg.norm(velocity))
