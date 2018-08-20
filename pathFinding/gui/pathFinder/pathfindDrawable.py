from engine.geometry.obstacle.arcFinder.arcSegmentFinder import ArcSegmentFinder
from engine.geometry.obstacle.intersectionDetector.pyPathIntersectionDetector import PyPathIntersectionDetector
from engine.geometry.obstacle.pathSegment import calcSegmentsPointDebug
from gui import Drawable
import gui


class PathFindDrawable(Drawable):

    def __init__(self, params, vehicle, scenario):
        self.scenario = scenario
        
        # TODO: Should create an obstacle course.  Move code to do this elsewhere.
        self._pathSegmentFinder = ArcSegmentFinder(vehicle.acceleration, params.nfzTargetOffset)
        self._pathIntersectionDetector = PyPathIntersectionDetector(params.nfzBufferWidth)
        self._pathIntersectionDetector.setInitialState(scenario.boundaryPoints, scenario.noFlyZones)
        self._pathIntersectionDetector.setDynamicNoFlyZones(scenario.dynamicNoFlyZones)  
        self._pathSegmentFinder.setInitialState(scenario.boundaryPoints, scenario.noFlyZones)
        self._pathSegmentFinder.setDynamicNoFlyZones(scenario.dynamicNoFlyZones)

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
        self._pathIntersectionDetector.draw(canvas, time=drawTime)
        self._pathSegmentFinder.draw(canvas, time=drawTime)
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
