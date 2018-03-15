from Tkinter import Canvas

import numpy as np

import constants
import gui
from findPathDynamic.gridHeap import GridHeap, Heap
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
        self._solution = None

        # Dynamic properties used for processing and display
        self._gridHeap = GridHeap(acceptanceThreshold, numBins, x, y, width, height)
        self._goalHeap = Heap()

        self._futureObstacleCourse = None
        self._currentVertex = None
        self._currentPaths = []
        self._processedVertices = []

        # TODO: We assume we start with [0,0] velocity, which is considered compatible (in terms of turning) with all other velocities.
        initialVelocity = np.array([0.0, 0.0], np.double)
        startVertex = Vertex(self._start, 0.0, initialVelocity)
        self._gridHeap.push(startVertex.position, startVertex.time, startVertex)
        self._numQueuedVertices = 1

    def step(self):
        if self._solution is None:
            lowestCostSolution = float("inf")
        else:
            lowestCostSolution = self._solution.time

        (lowestPossibleTimeCost, next) = self._gridHeap.popWithCost()
        if next is None:
            return False

        while lowestCostSolution < lowestPossibleTimeCost:
            (lowestPossibleTimeCost, next) = self._gridHeap.popWithCost()
            if next is None:
                return False

        self._currentVertex = next

        self._futureObstacleCourse = self._obstacleCourse.getFutureCopy(self._currentVertex.time)

        if not self._futureObstacleCourse.doesLineIntersect(self._currentVertex.position, self._goal,
                                                            self._constantSpeed):
            (travelTime, direction) = calcs.calcTravelTimeAndDirection(self._currentVertex.position, self._goal,
                                                                       self._constantSpeed)

            goalVertex = Vertex(self._goal,
                                self._currentVertex.time + travelTime,
                                direction * self._constantSpeed,
                                self._currentVertex)
            self._goalHeap.push(goalVertex.time, goalVertex)
            self._solution = self._goalHeap.getTop()
            lowestCostSolution = self._solution.time
        self._currentPaths = self._futureObstacleCourse.findStraightPathsToVertices(self._currentVertex.position,
                                                                                    self._constantSpeed,
                                                                                    lambda path:
                                                                                    isTurnLegal(
                                                                                        self._currentVertex.velocity,
                                                                                        path.velocity,
                                                                                        self._constantSpeed))

        for path in self._currentPaths:
            # TODO: convert remaining math to nparray
            destination = np.array(path.destination, np.double)
            newVertex = Vertex(destination,
                               self._currentVertex.time + path.time,
                               path.velocity,
                               self._currentVertex)
            lowestPossibleTimeCost = newVertex.time + self.heuristic(newVertex.position)

            # Don't push new items if we already know that an existing solution is guarenteed to be better
            if lowestPossibleTimeCost < lowestCostSolution:
                self._gridHeap.push(newVertex.position, lowestPossibleTimeCost, newVertex)

        self._processedVertices.append(self._currentVertex)
        print len(self._gridHeap)
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
            (closestPoint, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance,
                                                                   (self.fp._currentVertex, goalVertex))

        obstacleCourse = self.fp._obstacleCourse.getFutureCopy(drawTime)
        obstacleCourse.draw(canvas, color=obstacleColor)

        gui.draw.drawPoint(canvas, self.fp._start, color="black")
        gui.draw.drawPoint(canvas, self.fp._goal, color="black")

        for path in self.fp._currentPaths:
            velocity = path.velocity
            destination = path.destination
            gui.draw.drawLine(canvas, self.fp._currentVertex.position, destination, color=lineOfSightColor)

        for vertex in self.fp._processedVertices:
            vertex.draw(canvas, color=vertexColor)
            vertex.drawEdge(canvas, color=pathColor, width=2.0)

        if not self.fp._currentVertex is None:
            self.fp._currentVertex.drawPath(canvas, color="orange", width=4.0)

        if not goalVertex is None:
            goalVertex.drawPath(canvas, color="purple", width=4.0)

        if not closestPoint is None:
            gui.draw.drawPoint(canvas, closestPoint, color="orange")
