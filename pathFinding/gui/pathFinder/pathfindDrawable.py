from engine.geometry.pathSegment.pathSegment import calcSegmentsPointDebug
from gui import Drawable
import gui


class PathFindDrawable(Drawable):

    def __init__(self, scenario):
        self.scenario = scenario
        self._pastPathSegments = []
        self._futurePathSegments = []
        self._filteredPathSegments = []
        self._solutionWaypoints = []
        self._solutionPathSegments = []
        self._finished = False

    def updateDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        self._pastPathSegments = pastPathSegments
        self._futurePathSegments = futurePathSegments
        self._filteredPathSegments = filteredPathSegments

    def updateSolution(self, solutionsWaypoints, solutionPathSegments, finished):
        self._solutionWaypoints = solutionsWaypoints
        self._solutionPathSegments = solutionPathSegments
        self._finished = finished
        
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
             pathColor="purple",
             solutionColor="green",
             showFiltered=False,
             **kwargs):

        if pointOfInterest is None:
            drawTime = 0.0
        else:
            (pointOfInterest, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance)
            
        gui.draw.drawScenario(canvas, self.scenario, time=drawTime)

        if pointOfInterest is not None:
            gui.draw.drawPoint(canvas, pointOfInterest, color="cyan", outline="black", width=1.5, radius=1.0)
            
        if not self._finished:
            if showFiltered:
                for pathSegment in self._filteredPathSegments:
                    pathSegment.draw(canvas, color=lineOfSightColor, filtered=True)            
            for pathSegment in self._futurePathSegments:
                pathSegment.draw(canvas, color=lineOfSightColor)            
            for pathSegment in self._pastPathSegments:
                pathSegment.draw(canvas, color=pathColor, width=2.0)
            solutionWidth = 2
        else:
            solutionWidth = 4
            
        for pathSegment in self._solutionPathSegments:
            pathSegment.draw(canvas, color=solutionColor, width=solutionWidth)            

        for solutionWaypoint in self._solutionWaypoints:
            gui.draw.drawCircle(canvas, solutionWaypoint.position, solutionWaypoint.radius, color=solutionColor)
        
        return drawTime
