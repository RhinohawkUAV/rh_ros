from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException
from engine.geometry.obstacle.arcFinder.arcCriticalPoint import ArcCriticalPoint
import numpy as np


class PointTarget:

    def __init__(self, position, velocity):
        self.startPosition = position
        (self.direction, self.speed) = calcs.unitAndLength(velocity)

    def initialGuess(self, arc):
        if self.insideArc(arc.center, arc.radius):
            if self.speed == 0.0:
                # Target is unmoving and contained within the the vehicle's arc's circle.
                # It is impossible to reach this self.
                raise NoSolutionException
            criticalPoints = self.getCriticalPoints(arc)
            
            # Should be exactly 1 intersection
            if len(criticalPoints) != 1:
                raise NoSolutionException
            
            if criticalPoints[0].targetArc > criticalPoints[0].vehicleArc:
                # Vehicle reaches vehicle's arc's circle faster than target
                # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                raise NoSolutionException
            
            return criticalPoints[0].vehicleArc
            
        else:
            criticalPoints = self.getCriticalPoints(arc)
            if len(criticalPoints) > 1:
                # Path of target intersects the vehicle's arc's circle.  This requires special care in order to solve.
                if criticalPoints[0].targetArc < criticalPoints[0].vehicleArc:
                    # target falls inside vehicle's arc's circle faster than vehicle could arc there
                    
                    if criticalPoints[1].targetArc > criticalPoints[1].vehicleArc:
                        # Vehicle reaches vehicle's arc's circle faster than target
                        # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                        raise NoSolutionException
                    
                    return criticalPoints[1].vehicleArc
        
        (angle, solution) = self.getSolution(arc)
        return calcs.modAngle(angle - arc.start, 0.0)

    def getSolution(self, arc):
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

    def insideArc(self, center, radius):
        toCenter = self.startPosition - center
        return np.dot(toCenter, toCenter) < radius * radius
    
    def getCriticalPoints(self, arc):
        distances = calcs.rayIntersectCircle(self.startPosition, self.direction, arc.center, arc.radius)
        criticalPoints = []
        for distance in distances:
            if distance >= 0.0:
                intersectionPoint = self.startPosition + distance * self.direction
                targetTimeToIntersection = distance / self.speed
                
                criticalPoints.append(ArcCriticalPoint(vehicleArc=arc.arcToPoint(intersectionPoint),
                                                       targetArc=targetTimeToIntersection * arc.angularSpeed))
        return criticalPoints
