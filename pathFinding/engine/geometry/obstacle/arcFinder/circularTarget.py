from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCriticalPoint import ArcCriticalPoint
from engine.geometry.obstacle.arcFinder.target import Target
import numpy as np


class CircularTarget(Target):

    def __init__(self, position, velocity, radius, rotDirection):
        self.startPosition = position
        (self.direction, self.speed) = calcs.unitAndLength(velocity)
        self.radius = radius
        
        # There are two ways around the circle.  This specifies which solution is being found
        # The more clockwise (-1) or more CCW (1) solution
        self.rotDirection = rotDirection
        
        if self.rotDirection == 1:
            self.solutionIndex = 0
        else:
            self.solutionIndex = 1
        
    def notInitiallyReachable(self, arc):
        if arc.rotDirection == self.rotDirection:
            radius = arc.radius + self.radius
        else:
            radius = arc.radius - self.radius

        toCenter = self.startPosition - arc.center
        
        return np.dot(toCenter, toCenter) < radius * radius

    def getCriticalPoints(self, arc):
        outside = (arc.rotDirection == self.rotDirection)
        times = calcs.movingCircleCollision(self.startPosition, self.direction * self.speed, self.radius, arc.radius, outside)
        criticalPoints = []
        for time in times:
            intersectionPoint = self.startPosition + self.direction * (self.speed * time + self.radius)
            criticalPoints.append(ArcCriticalPoint(vehicleArc=arc.arcToPoint(intersectionPoint),
                                                   targetArc=arc.arcLength(time)))
        return criticalPoints

    def iterateSolution(self, arc):
        time = arc.arcTime()
        endAngle = arc.start + arc.length
        endAngleVec = calcs.unitVectorOfAngle(endAngle, arc.rotDirection)
        arcEndPoint = arc.center + arc.radius * endAngleVec
        
        velocityOfTarget = self.direction * self.speed
        targetStartPoint = self.startPosition + velocityOfTarget * time
            
        solutions = calcs.hitTargetCircleAtSpeed(arcEndPoint, arc.speed, targetStartPoint, velocityOfTarget, self.radius)
        solution = solutions[self.solutionIndex]
        if solution is None:
            raise NoSolutionException
        angle = arc.angleOfVelocity(solution.velocity)
        return angle, solution

