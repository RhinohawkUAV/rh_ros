import testmod

from engine.geometry import LineSegment, calcs
import numpy as np


class ObstacleLineSegment(LineSegment):
    """
    An extension to the standard LineSegment class which includes a velocity
    """

    def __init__(self, p1, p2, velocity):
        LineSegment.__init__(self, p1, p2)
        self.velocity = np.array(velocity, np.double)

    def checkPathIntersectsLine(self, startTime, startPoint, velocity, pathTime):
        """
        Does a path from startPoint to endPoint, at the given speed intersect?
        """
        return self.checkPathIntersectsLinePy(startTime, startPoint, velocity, pathTime)

    def checkPathIntersectsLineC(self, startTime, startPoint, endPoint, speed):
        """
        Does a path from startPoint to endPoint, at the given speed intersect?
        """
        # TODO: How are float64s getting in here?!?!  Uncomment to activate C calculations.
#         result = testmod.intersectPathAndLine(self, float(startTime), startPoint, endPoint, float(speed))
#         return result == 1
        return False
    
    def checkPathIntersectsLinePy(self, startTime, startPoint, velocity, pathTime):
        """
        Does a path from intersect line at any point in time?
        """
        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity = velocity - self.velocity

        # The new end point takes the same time to reach, but at a new offset heading
        endPoint = startPoint + velocity * pathTime

        # Given the start time, this line will have moved.  Alternately, we offset the start and end points in the
        # opposite direction
        offset = -self.velocity * startTime

        return self.checkLineIntersection(startPoint + offset, endPoint + offset)
    
