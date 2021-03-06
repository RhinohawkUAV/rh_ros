from engine.geometry import LineSegment
import gui.draw
import numpy as np


class LineSegmentObstacle(LineSegment):
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
        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity = velocity - self.velocity

        # The new end point takes the same time to reach, but at a new offset heading
        endPoint = startPoint + velocity * pathTime

        # Given the start time, this line will have moved.  Alternately, we offset the start and end points in the
        # opposite direction
        offset = -self.velocity * startTime

        return self.checkLineIntersection(startPoint + offset, endPoint + offset)
    
    def draw(self, visualizer, time=0.0, **kwargs):
        gui.draw.drawLine(visualizer, self.p1 + self.velocity * time, self.p2 + self.velocity * time, **kwargs)
        
