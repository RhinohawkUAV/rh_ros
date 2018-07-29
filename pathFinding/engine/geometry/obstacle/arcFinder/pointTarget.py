from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCriticalPoint import ArcCriticalPoint
from engine.geometry.obstacle.arcFinder.target import Target
import numpy as np


class PointTarget(Target):

    def __init__(self, position, velocity):
        self.startPosition = position
        (self.direction, self.speed) = calcs.unitAndLength(velocity)

    def notInitiallyReachable(self, arc):
        toCenter = self.startPosition - arc.center
        return np.dot(toCenter, toCenter) < arc.radius * arc.radius
    
    def getCriticalPoints(self, arc):
        distances = calcs.getRayCircleIntersections(self.startPosition, self.direction, arc.center, arc.radius)
        criticalPoints = []
        for distance in distances:
            if distance >= 0.0:
                intersectionPoint = self.startPosition + distance * self.direction
                targetTimeToIntersection = distance / self.speed
                
                criticalPoints.append(ArcCriticalPoint(vehicleArc=arc.arcToPoint(intersectionPoint),
                                                       targetArc=targetTimeToIntersection * arc.angularSpeed))
        return criticalPoints

    def iterateSolution(self, arc):
        time = arc.arcTime()
        endAngle = arc.start + arc.length
        endAngleVec = calcs.unitVectorOfAngle(endAngle, arc.rotDirection)
        arcEndPoint = arc.center + arc.radius * endAngleVec
        
        velocityOfTarget = self.direction * self.speed
        targetStartPoint = self.startPosition + velocityOfTarget * time
        solution = calcs.hitTargetAtSpeed(arcEndPoint, arc.speed, targetStartPoint, velocityOfTarget)
        if solution is None:
            raise NoSolutionException
        angle = arc.angleOfVelocity(solution.velocity)
        return angle, solution

