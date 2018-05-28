import math
from constants import NO_FLY_ZONE_POINT_OFFSET
from defaultObstacleData import DefaultObstacleData
from engine.geometry import calcs, arc
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
    def createPathSegment(self, startTime, startPoint, startVelocity, targetPoint, velocityOfTarget):
#         try:
#             arcFinder = ArcFinder(startPoint, startVelocity, targetPoint, velocityOfTarget, 1.0, self.acceleration)
#             arcFinder.solve()
#             return ArcPathSegment(startTime, arcFinder.totalTime, arcFinder.endPoint, arcFinder.finalVelocity,
#                                   arcFinder.speed, arcFinder.arc)
#         except NoSolutionException:
#             return None

        try:
            arcFinderCCW = ArcFinder(startPoint, startVelocity, targetPoint, velocityOfTarget, 1.0, self.acceleration)
            arcFinderCCW.solve()
            timeCCW = arcFinderCCW.totalTime
        except NoSolutionException:
            timeCCW = float("inf")
 
        try:
            arcFinderCW = ArcFinder(startPoint, startVelocity, targetPoint, velocityOfTarget, -1.0, self.acceleration)
            arcFinderCW.solve()
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
 
        return ArcPathSegment(startTime, arcFinder.totalTime, arcFinder.endPoint, arcFinder.finalVelocity,
                              arcFinder.speed, arcFinder.arc)


# TODO: Move to calcs module
class NoSolutionException(BaseException):
    pass


class ArcFinder:

    def __init__(self, startPoint, velocity, targetPoint, velocityOfTarget, rotDirection,
                 acceleration):
        self.startPoint = startPoint
        self.speed = np.linalg.norm(velocity)
        
        # TODO: Should move check to higher level
        if self.speed == 0.0 or acceleration == 0.0:
            raise NoSolutionException
        
        self.targetPoint = targetPoint
        
        self.speedOfTarget = np.linalg.norm(velocityOfTarget)
        if self.speedOfTarget > 0.0:
            self.directionOfTarget = velocityOfTarget / self.speedOfTarget
        
        self.velocityOfTarget = velocityOfTarget
        self.rotDirection = rotDirection

        self.arc = arc.createArc(self.startPoint, velocity, acceleration, rotDirection)
        self.totalTime = 0.0
        self.arcTime = 0.0
        self.endPoint = None
        self.finalVelocity = None

    def findArcIntersectionInfo(self, distance):
        intersectionPoint = self.targetPoint + distance * self.directionOfTarget
        intersectionTime = distance / self.speedOfTarget
        intersectionAngle = calcs.angleOfVector(intersectionPoint - self.arc.center, self.rotDirection)
        arcLengthToIntersection = calcs.modAngle(intersectionAngle, self.arc.start) - self.arc.start
        arcTime = arcLengthToIntersection * self.arc.radius / self.speed
        return (intersectionTime, arcTime)

    def calcInitialGuess(self):
        if self.speedOfTarget == 0.0:
            # Target is unmoving and contained within the the vehicle's arc's circle.
            # It is impossible to reach this target.
            toCenter = self.targetPoint - self.arc.center
            if np.dot(toCenter, toCenter) < self.arc.radius * self.arc.radius:
                raise NoSolutionException
        else:
            distances = calcs.rayIntersectCircle(self.targetPoint, self.directionOfTarget, self.arc.center, self.arc.radius)
            if len(distances) > 0:
                # Path of target intersects the vehicle's arc's circle.  This requires special care in order to solve.
                (intersectionTime, arcTime) = self.findArcIntersectionInfo(distances[0])
                if intersectionTime < arcTime and len(distances) > 1:
                    # Either target reaches vehicle's arc's circle faster than vheicle could get there OR
                    # intersectionTime is negative, meaning target is already within/past circle.
                    (intersectionTime, arcTime) = self.findArcIntersectionInfo(distances[1])
                    if arcTime < intersectionTime :
                        # Vehicle reaches vehicle's arc's circle faster than target
                        # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                        raise NoSolutionException
                    if intersectionTime > 0.0:
                        # Target is exiting the vehicle's arc's circle before the vehicle can get there.
                        # This allows us to iterate to a solution
                        return intersectionTime * self.speed / self.arc.radius
        
        solution = calcs.hitTargetAtSpeed(self.startPoint, self.speed, self.targetPoint, self.velocityOfTarget)
        if solution is None:
            raise NoSolutionException
        angle = calcs.angleOfVector(solution.velocity, self.rotDirection)
        angle -= math.pi / 2.0
        return calcs.modAngleUnsigned(angle - self.arc.start)
        
    def solve(self):
        initialArcLength = self.calcInitialGuess()
        self.arc.setLength(initialArcLength)
        self.arcTime = self.arc.length * self.arc.radius / self.speed
        
        iteration = 0
        while iteration < MAX_ITERATIONS:
            newTarget = self.targetPoint + self.velocityOfTarget * self.arcTime
            solution = calcs.hitTargetAtSpeed(self.arc.endPoint, self.speed, newTarget, self.velocityOfTarget)
            if solution is None:
                raise NoSolutionException
            angle = calcs.angleOfVector(solution.velocity, self.rotDirection)
            angle -= math.pi / 2.0
            
            angleDiff = calcs.modAngleSigned(angle - (self.arc.start + self.arc.length))
            if math.fabs(angleDiff) <= MAX_ANGLE_ERROR:
                self.totalTime = solution.time + self.arcTime
                self.endPoint = solution.endPoint
                self.finalVelocity = solution.velocity
                return
            self.iterateFindArc(angleDiff)
            iteration += 1
        raise NoSolutionException

    def iterateFindArc(self, angleDiff):
        self.arc.setLength(self.arc.length + angleDiff)
        if self.arc.length < 0.0:
            raise NoSolutionException
        self.arcTime = self.arc.length * self.arc.radius / self.speed
   
