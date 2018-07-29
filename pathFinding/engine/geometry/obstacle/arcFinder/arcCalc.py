import math

from engine.geometry import calcs
from engine.geometry.arc import Arc


class ArcCalc(Arc):

    def __init__(self, rotDirection, radius, center, start, length, speed):
        Arc.__init__(self, rotDirection, radius, center, start, length)
        self.speed = speed
        self.angularSpeed = speed / self.radius
    
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
