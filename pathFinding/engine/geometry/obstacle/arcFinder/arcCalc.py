import math

from engine.geometry import calcs
from engine.geometry.arc import Arc


def create(startPoint, speed, unitVelocity, rotDirection, acceleration):
    arcRadius = speed * speed / acceleration
    fromCenterDir = -rotDirection * calcs.CCWNorm(unitVelocity)
    fromCenterToStart = fromCenterDir * arcRadius
    center = startPoint - fromCenterToStart
    arcStart = calcs.angleOfVector(fromCenterToStart, rotDirection)
    return ArcCalc(rotDirection, arcRadius, center, arcStart, 0.0, speed, speed / arcRadius)


class ArcCalc(Arc):

    def __init__(self, rotDirection, radius, center, start, length, speed, angularSpeed):
        Arc.__init__(self, rotDirection, radius, center, start, length)
        self.speed = speed
        self.angularSpeed = angularSpeed       
   
    def angleOfVelocity(self, velocity):
        return calcs.angleOfVector(velocity, self.rotDirection) - math.pi / 2.0

    def angleOfPoint(self, point):
        """
        Angle of point relative to center of arc.
        """
        return calcs.angleOfVector(point - self.center, self.rotDirection)

    def arcToPoint(self, point):
        """
        How far to rotate from the arc start angle to get to the angle of the point.  Resulting arc length in range [0,pi*2)
        """
        angle = self.angleOfPoint(point)
        return calcs.modAngle(angle, self.start) - self.start
    
    def arcTime(self, arcLength=None):
        if arcLength is None:
            arcLength = self.length
        return arcLength / self.angularSpeed
    
    def arcLength(self, time):
        return time * self.angularSpeed

    def endInfo(self):
        end = self.start + self.length
        endVec = calcs.unitVectorOfAngle(end, self.rotDirection)
        endPoint = self.center + self.radius * endVec
        endDirection = self.rotDirection * calcs.CCWNorm(endVec)
        return (endPoint, endDirection)

    def copy(self):
        return ArcCalc(self.rotDirection, self.radius, self.center, self.start, self.length, self.speed, self.angularSpeed)
    
