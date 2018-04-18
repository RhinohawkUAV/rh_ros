from Tkinter import Canvas

import numpy as np

import gui
from gui import Drawable


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
                pathSegment = currentVertex.pathSegment
                (closestPathPoint, distance, timeParametric) = pathSegment.calcPointDebug(point)
                if distance < shortestDistance:
                    shortestDistance = distance
                    closestPoint = closestPathPoint
                    time = timeParametric * currentVertex.timeToVertex + \
                           (1 - timeParametric) * previousVertex.timeToVertex
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
            searchPaths = []

            if not self.fp.isDone():
                searchPaths.append(self.fp._currentVertex)
            if not self.fp._solution is None:
                searchPaths.append(self.fp._solution)
            (closestPoint, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance, searchPaths)

        obstacleCourse = self.fp._obstacleCourse.getFutureCopy(drawTime)
        obstacleCourse.draw(canvas, color=obstacleColor, drawVectors=False)

        gui.draw.drawPoint(canvas, self.fp._start, color="black")
        gui.draw.drawPoint(canvas, self.fp._goal, color="black")

        for vertex in self.fp._vertexQueue:
            vertex.draw(canvas, color=vertexColor)
            # vertex.drawEdge(canvas, color=pathColor, width=2.0)

        if not self.fp.isDone():
            self.fp._currentVertex.drawPath(canvas, color="orange", width=4.0)
            for pathSegment in self.fp._pathSegments:
                velocity = pathSegment.endVelocity
                destination = pathSegment.endPoint
                gui.draw.drawLine(canvas, self.fp._currentVertex.position, destination, color=lineOfSightColor)

        if not self.fp._solution is None:
            self.fp._solution.drawPath(canvas, color="purple", width=4.0)

        if not closestPoint is None:
            gui.draw.drawPoint(canvas, closestPoint, color="orange")

    def printStats(self):
        if self.fp._computeTime > 0.0:
            print str(self.fp._computeTime) + " , " + \
                  str(self.fp._findStraightPathsComputeTime / self.fp._computeTime) + " , " + \
                  str(self.fp._queueComputeTime / self.fp._computeTime)
