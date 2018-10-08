from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder import arcCalc
from engine.geometry.obstacle.arcFinder.arcCalc import ArcCalc
from engine.geometry.obstacle.arcFinder.arcFinder import ArcFinder
from engine.geometry.obstacle.arcFinder.arcPathSegment import ArcPathSegment
from engine.geometry.obstacle.arcFinder.circularArcTarget import CircularArcTarget
from engine.geometry.obstacle.arcFinder.vertexArcTarget import VertexArcTarget
from engine.geometry.obstacle.pathSegmentFinder import PathSegmentFinder
from utils import profile

# TODO: Move to calcs
_rotDirections = (-1.0, 1.0)


class ArcSegmentFinder(PathSegmentFinder):

    def __init__(self, params, vehicle):
        PathSegmentFinder.__init__(self, params, vehicle)

    def _createCircularTarget(self, center, radius, velocity):
        return CircularArcTarget(center, velocity, radius, self.params.nfzBufferWidth, self.params.nfzTargetOffset)

    def _createVertexTarget(self, vertexPosition, velocity, vertexNormal, vertexAngle):
        return VertexArcTarget(vertexPosition, velocity, vertexNormal, vertexAngle)

    def stall(self, startTime, startPoint, startSpeed, startUnitVelocity, stallTime):
        # How far to loop (angle) is based on the requested stall time, speed and acceleration
        loopLength = stallTime * self.vehicle.acceleration / startSpeed
 
        pathSegments = []
        for rotDirection in _rotDirections:
            arc = arcCalc.create(startPoint, startSpeed, startUnitVelocity, rotDirection, self.vehicle.acceleration)
            arc.length = loopLength
            pathSegments.append(ArcPathSegment(startTime, arc, 0.0, 0))
        
        return pathSegments

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget, legalRotDirection):
        
        # TODO: Should order based on shorter time.  This actually matters as the shorter time one will be explored first
        # (they are not very unique)
        
        if legalRotDirection == 0:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.vehicle.acceleration),
                          ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.vehicle.acceleration)]
        else:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, legalRotDirection, self.vehicle.acceleration)]
        pathSegments = []
        # In this case, the last 2 arguments are meaningless
        target = VertexArcTarget(targetPoint, velocityOfTarget, None, 0.0)
        target.update(startTime)
        
        for arcFinder in arcFinders:
            try:
                pathSegments.append(arcFinder.solve(target, startTime))
            except NoSolutionException:
                pass 
        return pathSegments

    # TODO: Merge logic into parent class
    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection):
        pathSegments = self._findStaticPathSegments(startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection)
        pathSegments.extend(self._findDynamicPathSegments(startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection))
        return pathSegments

    @profile.accumulate("Find Arc Point")
    def _findStaticPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection):
        if legalRotDirection == 0:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.vehicle.acceleration),
                          ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.vehicle.acceleration)]
        else:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, legalRotDirection, self.vehicle.acceleration)]
            
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
        return pathSegments

    @profile.accumulate("Find Arc Circle")
    def _findDynamicPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection):
        if legalRotDirection == 0:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.vehicle.acceleration),
                          ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.vehicle.acceleration)]
        else:
            arcFinders = [ArcFinder(startPoint, startSpeed, startUnitVelocity, legalRotDirection, self.vehicle.acceleration)]
                 
        pathSegments = []
        for target in self.circularTargets:
            target.update(startTime)
            for targetRotDirection in _rotDirections:
                for arcFinder in arcFinders:
                    target.setAvoidanceRotDirection(targetRotDirection)
                    try:
                        pathSegments.append(arcFinder.solve(target, startTime))
                    except NoSolutionException:
                        pass
        return pathSegments

    def draw(self, canvas, time=0.0, **kwargs):
        for target in self.vertexTargets:
            target.update(time)
            target.draw(canvas)
            
        for target in self.circularTargets:
            target.update(time)
            target.draw(canvas)
