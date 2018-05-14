from Tkinter import Canvas

from engine.geometry.pathSegment.pathSegment import calcSegmentsPointDebug
from gui import Drawable
import gui
from gui.editor.pathSegmentTester.obstacleDebug import ObstacleCourseDebug


class PathFindDrawable(Drawable):

    def __init__(self, scenario):
        self._obstacleCourseDebug = ObstacleCourseDebug(scenario.boundaryPoints, scenario.noFlyZones)
        self._pastPathSegments = []
        self._futurePathSegments = []
        self._filteredPathSegments = []
        self._solutionPathSegments = []
        self._finished = False

    def updateDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pastPathSegments = pastPathSegments
        self._futurePathSegments = futurePathSegments
        self._filteredPathSegments = filteredPathSegments

    def updateSolution(self, solutionPathSegments, finished):
        self._solutionPathSegments = solutionPathSegments
        self._finished = finished
        self._pastPathSegments = []
        self._futurePathSegments = []
        self._filteredPathSegments = []
    
    def findClosestPointOnPath(self, point, snapDistance):
        pathSegments = []
        pathSegments.extend(self._pastPathSegments)
        pathSegments.extend(self._futurePathSegments)
        pathSegments.extend(self._solutionPathSegments)
        (closestSegmentIndex, closestPoint, minimumDistance, closestTime) = calcSegmentsPointDebug(point, pathSegments,
                                                                                                   snapDistance)
        if closestSegmentIndex is not None:
            return (closestPoint, closestTime)
        else:
            return (point, 0.0)
        
    def draw(self, canvas, pointOfInterest=None, snapDistance=float("inf"), obstacleColor="black",
             lineOfSightColor="blue",
             vertexColor="green",
             pathColor="red",
             solutionColor="purple",
             **kwargs):

        if pointOfInterest is None:
            drawTime = 0.0
        else:
            (pointOfInterest, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance)
            
        self._obstacleCourseDebug.draw(canvas, time=drawTime, boundaryColor="red", nfzColor="black")
        if pointOfInterest is not None:
            gui.draw.drawPoint(canvas, pointOfInterest, color="orange")

        for pathSegment in self._futurePathSegments:
            pathSegment.draw(canvas, color=lineOfSightColor)            
        for pathSegment in self._pastPathSegments:
            pathSegment.draw(canvas, color=pathColor, width=2.0)
            
        if self._finished:
            solutionWidth = 4
        else:
            solutionWidth = 2
            
        for pathSegment in self._solutionPathSegments:
            pathSegment.draw(canvas, color=solutionColor, width=solutionWidth)            

