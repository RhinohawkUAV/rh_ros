from bzrlib import inter
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


class ArcIntersection:
    """
    Represents the point at which a target completely descends inside an arc's circle OR first emerges.
    """

    def __init__(self, arcTime, targetTime):
        self.arcTime = arcTime
        self.targetTime = targetTime
    

class PointTarget:

    def __init__(self, position, direction, speed):
        self.startPosition = position
        self.direction = direction
        self.speed = speed

    def insideArc(self, center, radius):
        toCenter = self.startPosition - center
        return np.dot(toCenter, toCenter) < radius * radius
    
    def calcIntersections(self, center, radius, startAngle, rotDirection, vehicleSpeed):
        distances = calcs.rayIntersectCircle(self.startPosition, self.direction, center, radius)
        intersections = []
        for distance in distances:
            if distance >= 0.0:
                intersectionPoint = self.startPosition + distance * self.direction
                intersectionTime = distance / self.speed
                intersectionAngle = calcs.angleOfVector(intersectionPoint - center, rotDirection)
                arcLengthToIntersection = calcs.modAngle(intersectionAngle, startAngle) - startAngle
                arcTime = arcLengthToIntersection * radius / vehicleSpeed
                intersections.append(ArcIntersection(arcTime, intersectionTime))
        return intersections

    
class ArcFinder:

    def __init__(self, startPoint, startSpeed, unitVelocity, rotDirection, acceleration):
        # TODO: Should move check to higher level
        if startSpeed == 0.0 or acceleration == 0.0:
            raise NoSolutionException

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

    def calcInitialGuess(self, target):
        if target.insideArc(self.center, self.arcRadius):
            if target.speed == 0.0:
                # Target is unmoving and contained within the the vehicle's arc's circle.
                # It is impossible to reach this target.
                raise NoSolutionException
            intersections = target.calcIntersections(self.center, self.arcRadius, self.arcStart, self.rotDirection, self.speed)
            
            # Should be exactly 1 intersection
            if len(intersections) != 1:
                raise NoSolutionException
            
            if intersections[0].arcTime < intersections[0].targetTime:
                # Vehicle reaches vehicle's arc's circle faster than target
                # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                raise NoSolutionException
            
            return intersections[0].arcTime * self.speed / self.arcRadius
            
        else:
            intersections = target.calcIntersections(self.center, self.arcRadius, self.arcStart, self.rotDirection, self.speed)
            if len(intersections) > 1:
                # Path of target intersects the vehicle's arc's circle.  This requires special care in order to solve.
                if intersections[0].targetTime < intersections[0].arcTime:
                    # target falls inside vehicle's arc's circle faster than vehicle could arc there
                    
                    if intersections[1].targetTime > intersections[1].arcTime:
                        # Vehicle reaches vehicle's arc's circle faster than target
                        # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                        raise NoSolutionException
                    
                    return intersections[1].arcTime * self.speed / self.arcRadius
 
        (angleDiff, solution) = self.getSolution(target.startPosition, target.direction * target.speed)
        return calcs.modAngle(angleDiff, 0.0)

    def solve(self, targetStartPoint, velocityOfTarget):
        (targetUnitVelocity, speedOfTarget) = calcs.unitAndLength(velocityOfTarget)
        
        target = PointTarget(targetStartPoint, targetUnitVelocity, speedOfTarget)
        
        self.arcLength = self.calcInitialGuess(target)
        self.updateArcParameters()
        
        iteration = 0
        while iteration < MAX_ITERATIONS:
            (angleDiff, solution) = self.getSolution(targetStartPoint, velocityOfTarget)
            if math.fabs(angleDiff) <= MAX_ANGLE_ERROR:
                self.totalTime = solution.time + self.arcTime 
                self.lineEndPoint = solution.endPoint
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
