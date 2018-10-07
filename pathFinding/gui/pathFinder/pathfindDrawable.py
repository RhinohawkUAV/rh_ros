from engine.geometry.obstacle import obstacleCourse
from engine.geometry.obstacle.pathSegment import calcSegmentsPointDebug
from gui import Drawable
import gui


class PathFindDrawable(Drawable):

    def __init__(self, params, vehicle, scenario):
        self.scenario = scenario
        self._obstacleCourse = obstacleCourse.createObstacleCourse(params, vehicle, scenario)

        self._finished = False
        self._bestPath = None
        self._previousPathSegments = []
        self._futurePathSegments = []
        self._filteredPathSegments = []

    def update(self, finished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        self._finished = finished
        self._bestPath = bestPath
        if self._finished:
            self._previousPathSegments = []
            self._futurePathSegments = []
            self._filteredPathSegments = []
        else:
            self._previousPathSegments = previousPathSegments
            self._futurePathSegments = futurePathSegments
            self._filteredPathSegments = filteredPathSegments
        
    def findClosestPointOnPath(self, point, snapDistance, showFiltered):
        pathSegments = []
        pathSegments.extend(self._previousPathSegments)
        pathSegments.extend(self._futurePathSegments)
        if showFiltered:
            pathSegments.extend(self._filteredPathSegments)
        if self._bestPath is not None:
            pathSegments.extend(self._bestPath.pathSegments)
            
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
            (pointOfInterest, drawTime) = self.findClosestPointOnPath(pointOfInterest, snapDistance, showFiltered)
            
        gui.draw.drawScenario(canvas, self.scenario, time=drawTime)
        self._obstacleCourse.draw(canvas, time=drawTime)
        if pointOfInterest is not None:
            gui.draw.drawPoint(canvas, pointOfInterest, color="cyan", outline="black", width=1.5, radius=1.0)
            
        if not self._finished:
            if showFiltered:
                for pathSegment in self._filteredPathSegments:
                    pathSegment.draw(canvas, color=lineOfSightColor, filtered=True)            
            for pathSegment in self._futurePathSegments:
                pathSegment.draw(canvas, color=lineOfSightColor)
                if pathSegment.debug is not None:
                    gui.draw.drawText(canvas, pathSegment.endPoint , str(pathSegment.debug) + "\n" + str(pathSegment.endPoint))  
            for pathSegment in self._previousPathSegments:
                pathSegment.draw(canvas, color=pathColor, width=2.0)
            solutionWidth = 3
        else:
            solutionWidth = 5
        if self._bestPath is not None:
            for pathSegment in self._bestPath.pathSegments:
                pathSegment.draw(canvas, color=solutionColor, width=solutionWidth)            

            for solutionWaypoint in self._bestPath.pathWaypoints:
                gui.draw.drawCircle(canvas, solutionWaypoint.position, solutionWaypoint.radius, color=solutionColor)
        
        return drawTime
