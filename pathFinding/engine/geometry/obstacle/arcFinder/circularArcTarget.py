from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCriticalPoint import ArcCriticalPoint
from engine.geometry.obstacle.arcFinder.arcTarget import ArcTarget
from engine.geometry.obstacle.circularTarget import CircularTarget
import numpy as np


class CircularArcTarget(CircularTarget, ArcTarget):

    def __init__(self, startPosition, velocity, radius, offset):
        CircularTarget.__init__(self, startPosition, velocity, radius, offset)
        self.avoidanceRotDirection = None
        self.solutionIndex = None
        
    def notInitiallyReachable(self, arc):
        if arc.rotDirection == self.avoidanceRotDirection:
            radius = arc.radius - self.radius
        else:
            radius = arc.radius + self.radius

        toCenter = self.position - arc.center
        
        return np.dot(toCenter, toCenter) < radius * radius

    def getCriticalPoints(self, arc):
        outside = (arc.rotDirection != self.avoidanceRotDirection)
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
 
        solutions = calcs.passTargetCircleAtSpeed(arcEndPoint,
                                                 arc.speed,
                                                 self.getPosition(arc.arcTime()),
                                                 self.velocity,
                                                 self.radius,
                                                 self.outerRadius)

        solution = solutions[self.solutionIndex]
        if solution is None:
            raise NoSolutionException
        angle = arc.angleOfVelocity(solution.velocity)
        return angle, solution

    def setAvoidanceRotDirection(self, rotDirection):
        # There are two ways to avoid the circle.  This sets the direction which the vehicle will be avoiding the obstacle.
        # By navigating CW (-1) or CCW (1) around the circle.
        self.avoidanceRotDirection = rotDirection
        
        if self.avoidanceRotDirection == -1.0:
            self.solutionIndex = 0
        else:
            self.solutionIndex = 1
            
    def calcAvoidanceRotDirection(self, passingVelocity):
        return self.avoidanceRotDirection
