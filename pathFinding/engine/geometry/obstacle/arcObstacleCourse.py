from constants import MAX_ARC_LENGTH
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder import ArcFinder
from engine.geometry.obstacle.arcFinder.circularTarget import CircularTarget
from engine.geometry.obstacle.arcFinder.pointTarget import PointTarget
from engine.geometry.obstacle.defaultObstacleCourse import DefaultObstacleCourse
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
from utils import profile

_rotDirections = (-1.0, 1.0)


class ArcObstacleCourse(DefaultObstacleCourse):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """
    
    def __init__(self, acceleration, targetOffsetLength):
        DefaultObstacleCourse.__init__(self, targetOffsetLength)
        self.acceleration = acceleration

    @profile.accumulate("Find Arc")
    def createPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):

        target = PointTarget(targetPoint, velocityOfTarget)
        unfilteredSegments = []
        try:
            arcFinder = ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.acceleration)
            unfilteredSegments.append(arcFinder.solve(target, startTime))
        except NoSolutionException:
            pass
        try:
            arcFinder = ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.acceleration)
            unfilteredSegments.append(arcFinder.solve(target, startTime))
        except NoSolutionException:
            pass
        return self.sortFilter(unfilteredSegments)
        
    def createPathSegmentsToDynamicNoFlyZone(self, startTime, startPoint, startSpeed, startUnitVelocity, dynamicNoFlyZone):
        unfilteredSegments = []
        for arcRotDirection in _rotDirections:
            for targetRotDirection in _rotDirections:
                arcFinder = ArcFinder(startPoint, startSpeed, startUnitVelocity, arcRotDirection, self.acceleration)
                target = CircularTarget(dynamicNoFlyZone.center + startTime * dynamicNoFlyZone.velocity,
                                        dynamicNoFlyZone.velocity,
                                        dynamicNoFlyZone.radius + self.targetOffsetLength,
                                        targetRotDirection)
                try:
                    unfilteredSegments.append(arcFinder.solve(target, startTime))
                except NoSolutionException:
                    pass
        return self.sortFilter(unfilteredSegments)
        
    def sortFilter(self, unfilteredSegments):
        unfilteredSegments.sort(key=lambda arc: arc.elapsedTime)
        segments = []
        for segment in unfilteredSegments:
            if segment.arc.length < MAX_ARC_LENGTH:
                segments.append(segment)
                # TODO: Remove this and search all arcs (too slow for testing)
#                 break
                    
        return segments
