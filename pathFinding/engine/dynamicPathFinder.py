import time

import numpy as np

from constants import NO_FLY_ZONE_POINT_OFFSET
from engine.geometry.pathSegment.lineSegmentObstacleData import LineSegmentObstacleData
from engine.vertex import UniqueVertexQueue
from geometry import calcs
from gui.obstacleDebug.obstacleDebug import ObstacleCourseDebug
from vertex import Vertex


class DynamicPathFinder:
    def __init__(self, initialPathFindingInput, maximumSpeed):

        self._obstacleData = LineSegmentObstacleData(NO_FLY_ZONE_POINT_OFFSET)
        self._obstacleData.setInitialState(initialPathFindingInput)

        # Calculate bounding rectangle and use that for dimensions of the UniqueVertexQueue
        bounds = initialPathFindingInput.calcBounds()
        self._vertexQueue = UniqueVertexQueue(bounds[0], bounds[1], bounds[2], bounds[3], maximumSpeed)

        self._obstacleCourseDebug = ObstacleCourseDebug(initialPathFindingInput.boundaryPoints,
                                                        initialPathFindingInput.noFlyZones)

    def findPath(self, pointToPointInput):
        self.initFindPath(pointToPointInput)
        while not self.isDone():
            self.step()

    def initFindPath(self, pointToPointInput):
        self._start = np.array(pointToPointInput.startPosition, np.double)
        self._goal = np.array(pointToPointInput.targetPoints[0], np.double)
        velocity = np.array(pointToPointInput.startVelocity, np.double)
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

    def isDone(self):
        return self._vertexQueue.isEmpty()

    def step(self):
        self._computeTime -= time.time()
        self._currentVertex = self._vertexQueue.pop()
        if self._currentVertex.estimatedTimeThroughVertex > self._bestSolutionTime:
            self._computeTime += time.time()
            return False

        self._obstacleData.setQueryTime(self._currentVertex.timeToVertex)
        self.checkPathToGoal()
        self._findPathsTime -= time.time()
        (self._pathSegments, self._filteredPathSegments) = self._obstacleData.findPathSegments(
            self._currentVertex.position,
            self._currentVertex.velocity)
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
        pathSegment = self._obstacleData.findPathSegment(self._currentVertex.position,
                                                         self._currentVertex.velocity,
                                                         self._goal,
                                                         np.array((0, 0), np.double))
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
