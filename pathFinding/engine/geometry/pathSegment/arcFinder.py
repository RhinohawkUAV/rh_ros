import math

from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
import numpy as np

# TODO: Move to constants
MAX_ITERATIONS = 5

# There will be difference between an arc's exit velocity and the correct velocity vector.
# This represents the allowed angular error (in radians) and corresponds to missing a target
# 1km away by 1m
MAX_ANGLE_ERROR = 0.001

MIN_ANGLE_ERROR_COS = math.cos(MAX_ANGLE_ERROR)


class ArcFinder:

    def __init__(self, startPoint, startSpeed, unitVelocity, rotDirection, acceleration):
        # TODO: Should move check to higher level
        if startSpeed == 0.0 or acceleration == 0.0:
            raise NoSolutionException

        self.startPoint = startPoint
        self.lineStartPoint = startPoint
        self.endUnitVelocity = unitVelocity
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
        self.lineEndPoint = None
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
                self.lineEndPoint = solution.lineEndPoint
                self.finalVelocity = solution.velocity
                return
            self.arcLength += angleDiff
            self.updateArcParameters()
            iteration += 1
        raise NoSolutionException

    def getSolution(self, targetStartPoint, velocityOfTarget):
        targetStartPoint = targetStartPoint + velocityOfTarget * self.arcTime
        solution = calcs.hitTargetAtSpeed(self.lineStartPoint, self.speed, targetStartPoint, velocityOfTarget)
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
        arcEndVec = calcs.unitVectorOfAngle(arcEnd, self.rotDirection)
        self.lineStartPoint = self.center + self.arcRadius * arcEndVec
        self.endUnitVelocity = self.rotDirection * calcs.CCWNorm(arcEndVec)
