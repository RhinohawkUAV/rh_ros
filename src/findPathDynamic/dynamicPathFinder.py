from Tkinter import Canvas

import numpy as np

import constants
import gui
from findPathDynamic.gridHeap import GridHeap, MinHeap
from findPathDynamic.vertex import Vertex
from geometry import LineSeg
from geometry import calcs
from gui import Drawable


class DynamicPathFinder:
    def __init__(self, start, goal, constantSpeed, obstacleCourse, acceptanceThreshold, numBins, x, y, width, height):
        self._start = np.array(start, np.double)
        self._goal = np.array(goal, np.double)
        self._constantSpeed = constantSpeed
        self._obstacleCourse = obstacleCourse

        # Dynamic properties used for processing and display
        self._vertexQueue = GridHeap(acceptanceThreshold, numBins, x, y, width, height)

        # TODO: We keep track of every path that reaches the end currently.  No real need to do this except for debugging.
        self._goalHeap = MinHeap()

        self._futureObstacleCourse = None

        # TODO: We assume we start with [0,0] velocity, which is considered compatible (in terms of turning) with all other velocities.
        initialVelocity = np.array([0.0, 0.0], np.double)
        self._currentVertex = Vertex(position=self._start,
                                     velocity=initialVelocity,
                                     timeToVertex=0.0,
                                     estimatedTimeThroughVertex=self.heuristic(self._start))
        self._nextEdges = []
        self._processedVertices = []
        self._solution = None
        self._bestSolutionTime = float("inf")
        self.running = True

        self._vertexQueue.push(self._currentVertex)
        self._numQueuedVertices = 1

    def step(self):
        self._currentVertex = self._vertexQueue.pop()
        while (not self._currentVertex is None) and (
                self._currentVertex.estimatedTimeThroughVertex > self._bestSolutionTime):
            self._currentVertex = self._vertexQueue.pop()

        if self._currentVertex is None:
            self.running = False
            return False

        self._futureObstacleCourse = self._obstacleCourse.getFutureCopy(self._currentVertex.time)

        if not self._futureObstacleCourse.doesLineIntersect(self._currentVertex.position, self._goal,
                                                            self._constantSpeed):
            (time, direction) = calcs.calcTravelTimeAndDirection(self._currentVertex.position, self._goal,
                                                                 self._constantSpeed)

            goalVertex = Vertex(position=self._goal,
                                velocity=direction * self._constantSpeed,
                                timeToVertex=self._currentVertex.time + time,
                                estimatedTimeThroughVertex=self._currentVertex.time + time,
                                previousVertex=self._currentVertex)
            self._goalHeap.push(goalVertex.time, goalVertex)
            self._solution = self._goalHeap.getTop()
            self._bestSolutionTime = self._solution.time

        self._nextEdges = self._futureObstacleCourse.findStraightPathsToVertices(self._currentVertex.position,
                                                                                 self._constantSpeed,
                                                                                 lambda path:
                                                                                 isTurnLegal(
                                                                                     self._currentVertex.velocity,
                                                                                     path.velocity,
                                                                                     self._constantSpeed))

        for edge in self._nextEdges:
            # TODO: convert remaining math to nparray
            destination = np.array(edge.destination, np.double)
            timeToVertex = self._currentVertex.time + edge.time
            newVertex = Vertex(position=destination,
                               velocity=edge.velocity,
                               timeToVertex=timeToVertex,
                               estimatedTimeThroughVertex=timeToVertex + self.heuristic(destination),
                               previousVertex=self._currentVertex)

            # Don't push new items if we already know that an existing solution is guaranteed to be better
            if newVertex.estimatedTimeThroughVertex < self._bestSolutionTime:
                self._vertexQueue.push(newVertex)

        self._processedVertices.append(self._currentVertex)
        print len(self._vertexQueue)
        return True

    def heuristic(self, point):
        return calcs.calcTravelTime(point, self._goal, self._constantSpeed)

    def findPath(self):
        while self.step():
            pass


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


class DynamicPathFinderDrawable(Drawable):
    def __init__(self, fp):
        self.fp = fp

    def findClosestPointOnPath(self, point, snapDistance, pathEndVertices):

        point = np.array(point, np.double)
        shortestDistance = snapDistance
        closestPoint = None
        time = 0.0

        for currentVertex in pathEndVertices:
            if currentVertex is None:
                break
            previousVertex = currentVertex.previousVertex

            while not previousVertex is None:
                line = LineSeg(currentVertex.position, previousVertex.position)
                parametric = line.closestPointParametric(point)
                closestPointOnLinePoint = line.getParametricPoint(parametric)
                distance = np.linalg.norm(point - closestPointOnLinePoint)
                if distance < shortestDistance:
                    shortestDistance = distance
                    closestPoint = closestPointOnLinePoint
                    time = (1.0 - parametric) * currentVertex.time + parametric * previousVertex.time
                currentVertex = previousVertex
                previousVertex = currentVertex.previousVertex

        return (closestPoint, time)

    def draw(self, canvas, pointOfInterest=None, snapDistance=float("inf"), obstacleColor="black",
             lineOfSightColor="blue",
             vertexColor="green",
             pathColor="red",
             **kwargs):
        """
        Draw on the canvas with any modifiers stored in kwargs.
        :param canvas:
        :param kwargs:
        :return:
        """
        # type: (Canvas,**kwargs) -> None

        goalVertex = None
        if not self.fp._goalHeap.isEmpty():
            goalVertex = self.fp._goalHeap.getTop()

        if pointOfInterest is None:
            closestPoint = None
            drawTime = 0.0
        else:
            if self.fp.running:
                searchPaths = [self.fp._currentVertex, goalVertex]
            else:
                searchPaths = [goalVertex]
            (closestPoint, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance, searchPaths)

        obstacleCourse = self.fp._obstacleCourse.getFutureCopy(drawTime)
        obstacleCourse.draw(canvas, color=obstacleColor, drawVectors=False)

        gui.draw.drawPoint(canvas, self.fp._start, color="black")
        gui.draw.drawPoint(canvas, self.fp._goal, color="black")

        for vertex in self.fp._processedVertices:
            vertex.draw(canvas, color=vertexColor)
            vertex.drawEdge(canvas, color=pathColor, width=2.0)

        if self.fp.running:
            self.fp._currentVertex.drawPath(canvas, color="orange", width=4.0)
            for edge in self.fp._nextEdges:
                velocity = edge.velocity
                destination = edge.destination
                gui.draw.drawLine(canvas, self.fp._currentVertex.position, destination, color=lineOfSightColor)

        if not goalVertex is None:
            goalVertex.drawPath(canvas, color="purple", width=4.0)

        if not closestPoint is None:
            gui.draw.drawPoint(canvas, closestPoint, color="orange")
