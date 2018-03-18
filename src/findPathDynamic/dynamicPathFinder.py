import time

import numpy as np

import constants
from findPathDynamic.vertex import Vertex
from geometry import calcs


class DynamicPathFinder:
    def __init__(self, start, goal, constantSpeed, obstacleCourse, vertexQueue):
        self._start = np.array(start, np.double)
        self._goal = np.array(goal, np.double)
        self._constantSpeed = constantSpeed
        self._obstacleCourse = obstacleCourse

        self._vertexQueue = vertexQueue

        # Dynamic properties used for processing and display
        self._futureObstacleCourse = None

        # TODO: We assume we start with [0,0] velocity, which is considered compatible (in terms of turning) with all other velocities.
        initialVelocity = np.array([0.0, 0.0], np.double)
        self._currentVertex = Vertex(position=self._start,
                                     velocity=initialVelocity,
                                     timeToVertex=0.0,
                                     estimatedTimeThroughVertex=self.heuristic(self._start))
        self._straightPaths = []
        self._solution = None
        self._bestSolutionTime = float("inf")

        self._vertexQueue.push(self._currentVertex)
        self._numQueuedVertices = 1
        self._computeTime = 0.0
        self._findStraightPathsComputeTime = 0.0
        self._queueComputeTime = 0.0

    def isDone(self):
        return self._vertexQueue.isEmpty()

    def step(self):
        self._computeTime -= time.time()
        self._currentVertex = self._vertexQueue.pop()
        if self._currentVertex.estimatedTimeThroughVertex > self._bestSolutionTime:
            self._computeTime += time.time()
            return False

        self._futureObstacleCourse = self._obstacleCourse.getFutureCopy(self._currentVertex.timeToVertex)
        self.checkPathToGoal()
        self._findStraightPathsComputeTime -= time.time()
        self._straightPaths = self._futureObstacleCourse.findStraightPathsToVertices(self._currentVertex.position,
                                                                                     self._constantSpeed,
                                                                                     lambda path:
                                                                                     isTurnLegal(
                                                                                         self._currentVertex.velocity,
                                                                                         path.velocity,
                                                                                         self._constantSpeed))
        self._findStraightPathsComputeTime += time.time()

        for straightPath in self._straightPaths:
            # TODO: convert remaining math to nparray
            destination = np.array(straightPath.destination, np.double)
            timeToVertex = self._currentVertex.timeToVertex + straightPath.time

            newVertex = Vertex(position=destination,
                               velocity=straightPath.velocity,
                               timeToVertex=timeToVertex,
                               estimatedTimeThroughVertex=timeToVertex + self.heuristic(destination),
                               previousVertex=self._currentVertex)
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
        self._findStraightPathsComputeTime -= time.time()
        if not self._futureObstacleCourse.doesLineIntersect(self._currentVertex.position, self._goal,
                                                            self._constantSpeed):
            (travelTime, direction) = calcs.calcTravelTimeAndDirection(self._currentVertex.position, self._goal,
                                                                       self._constantSpeed)

            timeToGoal = self._currentVertex.timeToVertex + travelTime
            self._findStraightPathsComputeTime += time.time()

            if timeToGoal < self._bestSolutionTime:
                self._solution = Vertex(position=self._goal,
                                        velocity=direction * self._constantSpeed,
                                        timeToVertex=timeToGoal,
                                        estimatedTimeThroughVertex=timeToGoal,  # timeToVertex + 0
                                        previousVertex=self._currentVertex)

                self._bestSolutionTime = timeToGoal
        else:
            self._findStraightPathsComputeTime += time.time()

    def heuristic(self, point):
        return calcs.calcTravelTime(point, self._goal, self._constantSpeed)

    def findPath(self):
        while not self.isDone():
            self.step()


def isTurnLegal(velocity1, velocity2, speed):
    """
    Assumes all velocities have equal magnitude and only need their relative angle checked.
    :param velocity1:
    :param velocity2:
    :return:
    """
    if velocity1[0] == 0.0 and velocity1[1] == 0.0:
        return True
    cosAngle = np.dot(velocity1, velocity2) / (speed * speed)
    return cosAngle > constants.MAX_TURN_ANGLE_COS
