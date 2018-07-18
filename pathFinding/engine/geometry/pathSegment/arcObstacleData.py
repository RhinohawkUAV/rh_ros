import math

from defaultObstacleData import DefaultObstacleData
from engine.geometry.arc import Arc
from engine.geometry.calcs import NoSolutionException
from engine.geometry.pathSegment.arcFinder import ArcFinder
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
from utils import profile


class ArcObstacleData(DefaultObstacleData):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, acceleration, targetOffsetLength):
        DefaultObstacleData.__init__(self, targetOffsetLength)
        self.acceleration = acceleration

    @profile.accumulate("Find Arc")
    def createPathSegmentToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        try:
            arcFinderCCW = ArcFinder(startPoint, startSpeed, startUnitVelocity, 1.0, self.acceleration)
            arcFinderCCW.solve(targetPoint, velocityOfTarget)
            timeCCW = arcFinderCCW.totalTime
        except NoSolutionException:
            timeCCW = float("inf")
 
        try:
            arcFinderCW = ArcFinder(startPoint, startSpeed, startUnitVelocity, -1.0, self.acceleration)
            arcFinderCW.solve(targetPoint, velocityOfTarget)
            timeCW = arcFinderCW.totalTime
        except NoSolutionException:
            timeCW = float("inf")
 
        # TODO: Should return both arcs and check for collisions before choosing
        if timeCCW < timeCW:
            arcFinder = arcFinderCCW
        elif timeCW < float("inf"):
            arcFinder = arcFinderCW
        else:
            return None
 
        return ArcPathSegment(startTime, arcFinder.totalTime, arcFinder.lineStartPoint, arcFinder.lineEndPoint, arcFinder.speed, arcFinder.endUnitVelocity,
                              Arc(arcFinder.rotDirection, arcFinder.arcRadius, arcFinder.center, arcFinder.arcStart, arcFinder.arcLength))

