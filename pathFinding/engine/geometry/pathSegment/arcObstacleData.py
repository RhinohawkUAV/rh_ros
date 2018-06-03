import math

from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs, arc
from engine.geometry.arc import Arc
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
import numpy as np
from utils import profile

# TODO: Move to constants
MAX_ITERATIONS = 5

# There will be difference between an arc's exit velocity and the correct velocity vector.
# This represents the allowed angular error (in radians) and corresponds to missing a target
# 1km away by 1m
MAX_ANGLE_ERROR = 0.001

MIN_ANGLE_ERROR_COS = math.cos(MAX_ANGLE_ERROR)


class ArcObstacleData(DefaultObstacleData):
    """
    Basic implementation of ObstacleData which produces simple line segments.  This assumes that the vehicle travels
    at a constant speed and that it is only limited by a maximum turning angle, which ignores speed.
    """

    def __init__(self, acceleration, targetOffsetLength=NO_FLY_ZONE_POINT_OFFSET):
        DefaultObstacleData.__init__(self, targetOffsetLength)
        self.acceleration = acceleration

    @profile.accumulate("Find Arc")
    def createPathSegment(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
#         try:
#             arcFinder = ArcFinder(startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget, 1.0, self.acceleration)
#             arcFinder.solve()
#             return ArcPathSegment(startTime, arcFinder.totalTime, arcFinder.endPoint, arcFinder.finalVelocity,
#                                   arcFinder.speed, arcFinder.arc)
#         except NoSolutionException:
#             return None
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
 
        return ArcPathSegment(startTime, arcFinder.totalTime, arcFinder.endPoint, arcFinder.speed,
                              Arc(arcFinder.rotDirection, arcFinder.arcRadius, arcFinder.center, arcFinder.arcStart, arcFinder.arcLength))


# TODO: Move to calcs module
class NoSolutionException(BaseException):
    pass


class ArcFinder:

    def __init__(self, startPoint, startSpeed, unitVelocity, rotDirection, acceleration):
        # TODO: Should move check to higher level
        if startSpeed == 0.0 or acceleration == 0.0:
            raise NoSolutionException

        self.startPoint = startPoint
        self.arcEndPoint = startPoint
        self.speed = startSpeed
        self.rotDirection = rotDirection
        self.arcRadius = self.speed * self.speed / acceleration
        fromCenterDir = -rotDirection * calcs.CCWNorm(unitVelocity)
        fromCenterToStart = fromCenterDir * self.arcRadius
    
        self.center = startPoint - fromCenterToStart
        self.arcStart = calcs.angleOfVector(fromCenterToStart, rotDirection)

        self.arcLength = 0.0
        self.arcTime = 0.0
        self.totalTime = 0.0
        self.endPoint = None
        self.finalVelocity = None

    def findArcIntersectionInfo(self, targetStartPoint, distance, targetUnitVelocity, speedOfTarget):
        intersectionPoint = targetStartPoint + distance * targetUnitVelocity
        intersectionTime = distance / speedOfTarget
        intersectionAngle = calcs.angleOfVector(intersectionPoint - self.center, self.rotDirection)
        arcLengthToIntersection = calcs.modAngle(intersectionAngle, self.arcStart) - self.arcStart
        arcTime = arcLengthToIntersection * self.arcRadius / self.speed
        return (intersectionTime, arcTime)

    def calcInitialGuess(self, targetStartPoint, targetUnitVelocity, speedOfTarget):
        if speedOfTarget == 0.0:
            # Target is unmoving and contained within the the vehicle's arc's circle.
            # It is impossible to reach this target.
            toCenter = targetStartPoint - self.center
            if np.dot(toCenter, toCenter) < self.arcRadius * self.arcRadius:
                raise NoSolutionException
        else:
            distances = calcs.rayIntersectCircle(targetStartPoint, targetUnitVelocity, self.center, self.arcRadius)
            if len(distances) > 0:
                # Path of target intersects the vehicle's arc's circle.  This requires special care in order to solve.
                (intersectionTime, arcTime) = self.findArcIntersectionInfo(targetStartPoint, distances[0], targetUnitVelocity, speedOfTarget)
                if intersectionTime < arcTime and len(distances) > 1:
                    # Either target reaches vehicle's arc's circle faster than vheicle could get there OR
                    # intersectionTime is negative, meaning target is already within/past circle.
                    (intersectionTime, arcTime) = self.findArcIntersectionInfo(targetStartPoint, distances[1], targetUnitVelocity, speedOfTarget)
                    if arcTime < intersectionTime :
                        # Vehicle reaches vehicle's arc's circle faster than target
                        # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                        raise NoSolutionException
                    if intersectionTime > 0.0:
                        # Target is exiting the vehicle's arc's circle before the vehicle can get there.
                        # This allows us to iterate to a solution
                        return intersectionTime * self.speed / self.arcRadius
        return self.arcStart

    def solve(self, targetStartPoint, velocityOfTarget):
        (targetUnitVelocity, speedOfTarget) = calcs.unitAndLength(velocityOfTarget)
        self.arcLength = self.calcInitialGuess(targetStartPoint, targetUnitVelocity, speedOfTarget)
        self.updateArcParameters()
        
        iteration = 0
        while iteration < MAX_ITERATIONS:
            (angleDiff, solution) = self.getSolution(targetStartPoint, velocityOfTarget)
            if math.fabs(angleDiff) <= MAX_ANGLE_ERROR:
                self.totalTime = solution.time + self.arcTime
                self.endPoint = solution.endPoint
                self.finalVelocity = solution.velocity
                return
            self.arcLength += angleDiff
            self.updateArcParameters()
            iteration += 1
        raise NoSolutionException

    def getSolution(self, targetStartPoint, velocityOfTarget):
        targetStartPoint = targetStartPoint + velocityOfTarget * self.arcTime
        solution = calcs.hitTargetAtSpeed(self.arcEndPoint, self.speed, targetStartPoint, velocityOfTarget)
        if solution is None:
            raise NoSolutionException
        angle = calcs.angleOfVector(solution.velocity, self.rotDirection)
        angle -= math.pi / 2.0
        return (calcs.modAngleSigned(angle - (self.arcStart + self.arcLength)), solution)

    def updateArcParameters(self):
        if self.arcLength < 0.0:
            raise NoSolutionException
        self.arcTime = self.arcLength * self.arcRadius / self.speed
        arcEnd = self.arcStart + self.arcLength
        self.arcEndPoint = self.center + self.arcRadius * calcs.unitVectorOfAngle(arcEnd, self.rotDirection)
