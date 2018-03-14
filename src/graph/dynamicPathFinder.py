from Tkinter import Canvas

import numpy as np

import gui
from graph.gridHeap import GridHeap
from graph.vertex import Vertex
from gui import Drawable


def calcTimeCost(p1, p2, speed):
    return np.linalg.norm(p2 - p1) / speed


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
        self._gridHeap.push(startVertex.position, startVertex.timeCost, startVertex)

    def step(self):
        next = self._gridHeap.pop()
        if next is None:
            return False
        self._currentVertex = next

        self._futureObstacleCourse = self._obstacleCourse.getFutureCopy(self._currentVertex.timeCost)

        self._currentPaths = self._futureObstacleCourse.findPathsToVertices(self._currentVertex.position,
                                                                            self._constantSpeed)

        if not self._futureObstacleCourse.doesLineIntersect(self._currentVertex.position, self._goal,
                                                            self._constantSpeed):
            # TODO: Actually calculate this once we are using it
            bsVelocity = np.array([1.0, 1.0], np.double)
            self._currentPaths.append((bsVelocity, self._goal))

        for path in self._currentPaths:
            velocity = path[0]
            # TODO: convert remaining math to nparray
            destination = np.array(path[1], np.double)

            costToNextVertex = calcTimeCost(self._currentVertex.position, destination, self._constantSpeed)

            # TODO: Hack to not go to the same vertex you are already at.
            if costToNextVertex < 0.01:
                continue

            timeCost = self._currentVertex.timeCost + costToNextVertex

            newVertex = Vertex(destination, timeCost, velocity, self._currentVertex)

            estimatedTimeCost = newVertex.timeCost + self.heuristic(newVertex.position)
            self._gridHeap.push(newVertex.position, estimatedTimeCost, newVertex)

        self._processedVertices.append(self._currentVertex)
        return True

    def heuristic(self, point):
        return calcTimeCost(point, self._goal, self._constantSpeed)

    def findPath(self):
        while self.step():
            pass


class DynamicPathFinderDrawable(Drawable):
    def __init__(self, fp):
        self.fp = fp

    def draw(self, canvas, obstacleColor="black", lineOfSightColor="blue", vertexColor="green", pathColor="red",
             **kwargs):
        """
        Draw on the canvas with any modifiers stored in kwargs.
        :param canvas:
        :param kwargs:
        :return:
        """
        # type: (Canvas,**kwargs) -> None
        self.fp._obstacleCourse.draw(canvas, color=obstacleColor)

        gui.draw.drawPoint(canvas, self.fp._start, color="black")
        gui.draw.drawPoint(canvas, self.fp._goal, color="black")

        for path in self.fp._currentPaths:
            velocity = path[0]
            destination = path[1]
            gui.draw.drawLine(canvas, self.fp._currentVertex.position, destination, color=lineOfSightColor)

        for vertex in self.fp._processedVertices:
            vertex.draw(canvas, color=vertexColor)
            vertex.drawEdge(canvas, color=pathColor, width=2.0)

        self.fp._currentVertex.drawPath(canvas, color="orange", width=4.0)
