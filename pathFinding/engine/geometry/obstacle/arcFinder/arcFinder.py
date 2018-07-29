import math

from engine.geometry import calcs
from engine.geometry.arc import Arc
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCalc import ArcCalc
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

        self.lineStartPoint = startPoint
        self.endUnitVelocity = unitVelocity

        arcRadius = startSpeed * startSpeed / acceleration
        fromCenterDir = -rotDirection * calcs.CCWNorm(unitVelocity)
        fromCenterToStart = fromCenterDir * arcRadius
    
        center = startPoint - fromCenterToStart
        arcStart = calcs.angleOfVector(fromCenterToStart, rotDirection)
        self.arc = ArcCalc(rotDirection, arcRadius, center, arcStart, 0.0, startSpeed)

        self.arcTime = 0.0
        self.totalTime = 0.0
        self.lineEndPoint = None
        self.finalVelocity = None

    def solve(self, target):
        
        self.arc.length = target.initialGuess(self.arc)
        
        iteration = 0
        while iteration < MAX_ITERATIONS:
            (angle, solution) = target.iterateSolution(self.arc)
            angleDiff = calcs.modAngleSigned(angle - (self.arc.start + self.arc.length))
            
            if math.fabs(angleDiff) <= MAX_ANGLE_ERROR:
                self.arcTime = self.arc.arcTime()
                self.totalTime = solution.time + self.arcTime 
                self.lineEndPoint = solution.endPoint
                self.finalVelocity = solution.velocity
                arcEnd = self.arc.start + self.arc.length
                arcEndVec = calcs.unitVectorOfAngle(arcEnd, self.arc.rotDirection)
                self.lineStartPoint = self.arc.center + self.arc.radius * arcEndVec
                self.endUnitVelocity = self.arc.rotDirection * calcs.CCWNorm(arcEndVec)                
                return
            self.arc.length += angleDiff
            if self.arc.length < 0.0:
                raise NoSolutionException            
            iteration += 1
        raise NoSolutionException

