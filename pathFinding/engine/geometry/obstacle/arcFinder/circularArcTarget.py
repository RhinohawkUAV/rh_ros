from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCriticalPoint import ArcCriticalPoint
from engine.geometry.obstacle.arcFinder.arcTarget import ArcTarget
from engine.geometry.obstacle.circularTarget import CircularTarget
import numpy as np


class CircularArcTarget(CircularTarget, ArcTarget):

    def __init__(self, startPosition, velocity, radius):
        CircularTarget.__init__(self, startPosition, velocity, radius)
        self.rotDirection = None
        self.solutionIndex = None
    
    def setRotDirection(self, rotDirection):
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

        toCenter = self.position - arc.center
        
        return np.dot(toCenter, toCenter) < radius * radius

    def getCriticalPoints(self, arc):
        outside = (arc.rotDirection == self.rotDirection)
        times = calcs.movingCircleCollision(self.position, self.direction * self.speed, self.radius, arc.radius, outside)
        criticalPoints = []
        for time in times:
            intersectionPoint = self.position + self.direction * (self.speed * time + self.radius)
            criticalPoints.append(ArcCriticalPoint(vehicleArc=arc.arcToPoint(intersectionPoint),
                                                   targetArc=arc.arcLength(time)))
        return criticalPoints

    def iterateSolution(self, arc):
        endAngle = arc.start + arc.length
        endAngleVec = calcs.unitVectorOfAngle(endAngle, arc.rotDirection)
        arcEndPoint = arc.center + arc.radius * endAngleVec
 
        solutions = calcs.hitTargetCircleAtSpeed(arcEndPoint, arc.speed, self.getPosition(arc.arcTime()), self.velocity, self.radius)
        solution = solutions[self.solutionIndex]
        if solution is None:
            raise NoSolutionException
        angle = arc.angleOfVelocity(solution.velocity)
        return angle, solution
