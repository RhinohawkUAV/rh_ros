import math

from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder import arcCalc
from engine.geometry.obstacle.arcFinder.arcPathSegment import ArcPathSegment

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
        self.arc = arcCalc.create(startPoint, startSpeed, unitVelocity, rotDirection, acceleration)

    def solve(self, target, starTimeOffset):
        self.arc.length = target.initialGuess(self.arc)
        iteration = 0
        while iteration < MAX_ITERATIONS:
            (angle, solution) = target.iterateSolution(self.arc)
            angleDiff = calcs.modAngleSigned(angle - (self.arc.start + self.arc.length))
            self.arc.length += angleDiff
            if self.arc.length < 0.0:
                raise NoSolutionException            
            elif math.fabs(angleDiff) <= MAX_ANGLE_ERROR:
                return self.generateSolution(target, solution, starTimeOffset)
            iteration += 1
        raise NoSolutionException
  
    def generateSolution(self, target, solution, starTimeOffset):
        lineTime = solution.time
        return ArcPathSegment(starTimeOffset, self.arc.copy(), lineTime, target.calcAvoidanceRotDirection(solution.velocity))
    
