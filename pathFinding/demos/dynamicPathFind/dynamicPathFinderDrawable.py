from Tkinter import Canvas

import gui
from engine.geometry import calcSegmentsPointDebug
from gui import Drawable


class DynamicPathFinderDrawable(Drawable):
    def __init__(self, fp):
        self.fp = fp

    def getPathSegments(self, pathEndVertices):
        pathSegments = []
        pathStartTimes = []
        for currentVertex in pathEndVertices:
            if currentVertex is None:
                break
            previousVertex = currentVertex.previousVertex

            while not previousVertex is None:
                pathSegments.append(currentVertex.pathSegment)
                pathStartTimes.append(previousVertex.timeToVertex)
                currentVertex = previousVertex
                previousVertex = currentVertex.previousVertex
        return (pathSegments, pathStartTimes)

    def findClosestPointOnPath(self, point, snapDistance, pathEndVertices):
        (pathSegments, pathStartTimes) = self.getPathSegments(pathEndVertices)
        (closestSegmentIndex, closestPoint, minimumDistance, closestTime) = calcSegmentsPointDebug(point, pathSegments,
                                                                                                   snapDistance)
        if closestSegmentIndex is not None:
            return (closestPoint, pathStartTimes[closestSegmentIndex] + closestTime)
        else:
            return (None, 0.0)

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

        self.fp._obstacleCourseDebug.draw(canvas, time=drawTime, boundaryColor="red", nfzColor="black")

        gui.draw.drawPoint(canvas, self.fp._start, color="black")
        gui.draw.drawPoint(canvas, self.fp._goal, color="black")

        i = 0
        for vertex in self.fp._vertexQueue:
            vertex.draw(canvas, color=vertexColor)
            if i > 5:
                break

        if not self.fp.isDone():
            self.fp._currentVertex.drawPath(canvas, color="orange", width=4.0)
            for pathSegment in self.fp._pathSegments:
                pathSegment.draw(canvas, color=lineOfSightColor)

            # for pathSegment in self.fp._filteredPathSegments:
            #     pathSegment.draw(canvas, color=lineOfSightColor, filtered=True)

        if not self.fp._solution is None:
            self.fp._solution.drawPath(canvas, color="purple", width=4.0)

        if not closestPoint is None:
            gui.draw.drawPoint(canvas, closestPoint, color="orange")

    def printStats(self):
        if self.fp._computeTime > 0.0:
            print str(self.fp._computeTime) + " , " + \
                  str(self.fp._findStraightPathsComputeTime / self.fp._computeTime) + " , " + \
                  str(self.fp._queueComputeTime / self.fp._computeTime)
