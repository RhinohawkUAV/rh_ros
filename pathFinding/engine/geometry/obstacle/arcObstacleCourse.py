from constants import MAX_ARC_LENGTH
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder import ArcFinder
from engine.geometry.obstacle.arcFinder.pointTarget import PointTarget
from engine.geometry.obstacle.defaultObstacleCourse import DefaultObstacleCourse
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
from utils import profile


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
        try:
            arcFinderCCW = ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.acceleration)
            arcFinderCCW.solve(target)
            timeCCW = arcFinderCCW.totalTime
        except NoSolutionException:
            timeCCW = float("inf")
 
        try:
            arcFinderCW = ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.acceleration)
            arcFinderCW.solve(target)
            timeCW = arcFinderCW.totalTime
        except NoSolutionException:
            timeCW = float("inf")
 
        arcFinders = [] 
        if timeCCW < timeCW:
            arcFinders.append(arcFinderCCW)
            if timeCW < float("inf"):
                arcFinders.append(arcFinderCW)
        elif timeCW < float("inf"):
            arcFinders.append(arcFinderCW)
            if timeCCW < float("inf"):
                arcFinders.append(arcFinderCCW)

        segments = []
        for arcFinder in arcFinders:
            if arcFinder.arc.length < MAX_ARC_LENGTH:
                segment = ArcPathSegment(startTime,
                                         arcFinder.totalTime,
                                         arcFinder.lineStartPoint,
                                         arcFinder.lineEndPoint,
                                         startSpeed,
                                         arcFinder.endUnitVelocity,
                                         arcFinder.arc)
                segments.append(segment)
                # TODO: Remove this and search all arcs (too slow for testing)
                break

        return segments
