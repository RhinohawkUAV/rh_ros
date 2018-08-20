from constants import MAX_ARC_LENGTH
from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcFinder import ArcFinder
from engine.geometry.obstacle.arcFinder.circularArcTarget import CircularArcTarget
from engine.geometry.obstacle.arcFinder.vertexArcTarget import VertexArcTarget
from engine.geometry.obstacle.pathSegmentFinder import PathSegmentFinder
from gui.draw import DEFAULT_DASH
import gui.draw
from utils import profile

_rotDirections = (-1.0, 1.0)


class ArcSegmentFinder(PathSegmentFinder):

    def __init__(self, acceleration, targetOffset):
        PathSegmentFinder.__init__(self, targetOffset)
        self.acceleration = acceleration
        self.vertexTargets = []
        self.circularTargets = []        

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        self.circularTargets = []
        for dfnz in dynamicNoFlyZones:
            self.circularTargets.append(CircularArcTarget(dfnz.center, dfnz.velocity, dfnz.radius + self.targetOffset))
        
    def createVertexTarget(self, point, velocity, normal, pointAngle):
        self.vertexTargets.append(VertexArcTarget(point, velocity, normal, pointAngle))

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        pathSegments = []
        # In this case, the last 2 arguments are meaningless
        target = VertexArcTarget(targetPoint, velocityOfTarget, None, 0.0)
        target.update(startTime)
        for arcRotDirection in _rotDirections:
            try:
                arcFinder = ArcFinder(startPoint, startSpeed, startUnitVelocity, arcRotDirection, self.acceleration)
                pathSegments.append(arcFinder.solve(target, startTime))
            except NoSolutionException:
                pass 
        return self.sortFilter(pathSegments)

    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        pathSegments = self._findStaticPathSegments(startTime, startPoint, startSpeed, startUnitVelocity)
        pathSegments.extend(self._findDynamicPathSegments(startTime, startPoint, startSpeed, startUnitVelocity))
        return pathSegments

    @profile.accumulate("Find Arc Point")
    def _findStaticPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, -1, self.acceleration),
                      ArcFinder(startPoint, startSpeed, startUnitVelocity, 1, self.acceleration)]
        pathSegments = []
        for target in self.vertexTargets:
            target.update(startTime)
            if not calcs.arePointsClose(startPoint, target.position):
                for arcFinder in arcFinders:
                    try:
                        pathSegment = arcFinder.solve(target, startTime)
                        if target.testEntryVelocity(pathSegment.endSpeed * pathSegment.endUnitVelocity):
                            pathSegments.append(pathSegment)
                    except NoSolutionException:
                        pass 
        return self.sortFilter(pathSegments)

    @profile.accumulate("Find Arc Circle")
    def _findDynamicPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, -1, self.acceleration),
                      ArcFinder(startPoint, startSpeed, startUnitVelocity, 1, self.acceleration)]
         
        pathSegments = []
        for target in self.circularTargets:
            target.update(startTime)
            for targetRotDirection in _rotDirections:
                for arcFinder in arcFinders:
                    target.setRotDirection(targetRotDirection)
                    try:
                        pathSegments.append(arcFinder.solve(target, startTime))
                    except NoSolutionException:
                        pass
        return self.sortFilter(pathSegments)

    # TODO: Use Python Filter Lambda
    def sortFilter(self, unfilteredSegments):
#         unfilteredSegments.sort(key=lambda arc: arc.elapsedTime)
        segments = []
        for segment in unfilteredSegments:
            if segment.arc.length < MAX_ARC_LENGTH:
                segments.append(segment)
                    
        return segments

    def draw(self, canvas, time=0.0, **kwargs):
        for target in self.vertexTargets:
            target.update(time)
            gui.draw.drawPoint(canvas, target.position, 4)
            
        for target in self.circularTargets:
            target.update(time)
            gui.draw.drawCircle(canvas, target.position, target.radius, dash=DEFAULT_DASH)
