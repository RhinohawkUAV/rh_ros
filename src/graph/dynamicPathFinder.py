from Tkinter import Canvas

import numpy as np

import gui
from geometry import StraightPathSolution, LineSeg
from geometry import calcs
from graph.gridHeap import GridHeap
from graph.vertex import Vertex
from gui import Drawable


class DynamicPathFinder:
    def __init__(self, start, goal, constantSpeed, obstacleCourse, acceptanceThreshold, numBins, x, y, width, height):
        self._start = np.array(start, np.double)
        self._goal = np.array(goal, np.double)
        self._constantSpeed = constantSpeed
        self._obstacleCourse = obstacleCourse

        # Dynamic properties used for processing and display
        self._gridHeap = GridHeap(acceptanceThreshold, numBins, x, y, width, height)
        self._futureObstacleCourse = None
        self._currentVertex = None
        self._currentPaths = []
        self._processedVertices = []

        # We aren't using velocity for anything yet
        initialVelocity = np.array([1.0, 1.0], np.double)
        startVertex = Vertex(self._start, 0.0, initialVelocity)
        self._gridHeap.push(startVertex.position, startVertex.time, startVertex)

    def step(self):
        next = self._gridHeap.pop()
        if next is None:
            return False
        self._currentVertex = next

        self._futureObstacleCourse = self._obstacleCourse.getFutureCopy(self._currentVertex.time)

        self._currentPaths = self._futureObstacleCourse.findStraightPathsToVertices(self._currentVertex.position,
                                                                                    self._constantSpeed)

        if not self._futureObstacleCourse.doesLineIntersect(self._currentVertex.position, self._goal,
                                                            self._constantSpeed):
            (timeToGoal, direction) = calcs.calcTravelTimeAndDirection(self._currentVertex.position, self._goal,
                                                                       self._constantSpeed)

            self._currentPaths.append(StraightPathSolution(timeToGoal, direction * self._constantSpeed, self._goal))

        for path in self._currentPaths:
            timeToVertex = path.time
            velocity = path.velocity
            # TODO: convert remaining math to nparray
            destination = np.array(path.destination, np.double)

            # TODO: Hack to not go to the same vertex you are already at.
            if timeToVertex < 0.01:
                continue

            timeCost = self._currentVertex.time + timeToVertex

            newVertex = Vertex(destination, timeCost, velocity, self._currentVertex)

            estimatedTimeCost = newVertex.time + self.heuristic(newVertex.position)
            self._gridHeap.push(newVertex.position, estimatedTimeCost, newVertex)

        self._processedVertices.append(self._currentVertex)
        return True

    def heuristic(self, point):
        return calcs.calcTravelTime(point, self._goal, self._constantSpeed)

    def findPath(self):
        while self.step():
            pass


class DynamicPathFinderDrawable(Drawable):
    def __init__(self, fp):
        self.fp = fp

    def findClosestPointOnPath(self, point, snapDistance):
        currentVertex = self.fp._currentVertex
        if self.fp._currentVertex is None:
            return (None, 0.0)
        previousVertex = currentVertex.previousVertex

        point = np.array(point, np.double)
        closestPoint = None
        time = 0.0
        shortestDistance = snapDistance

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

        if pointOfInterest is None:
            closestPoint = None
            drawTime = 0.0
        else:
            (closestPoint, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance)

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

        if not closestPoint is None:
            gui.draw.drawPoint(canvas, closestPoint, color="orange")
