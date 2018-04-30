import numpy as np

from engine.geometry import LineSegment


class ObstacleLineSegment(LineSegment):
    """
    An extension to the standard LineSegment class which includes a velocity
    """

    def __init__(self, p1, p2, velocity):
        LineSegment.__init__(self, p1, p2)
        self.velocity = velocity

    def checkPathIntersectsLine(self, startTime, startPoint, endPoint, speed):
        """
        Does a path from startPoint to endPoint, at the given speed intersect?
        """
        direction = endPoint - startPoint
        distance = np.linalg.norm(direction)
        if distance == 0.0:
            return False

        # velocity vector - has magnitude in speed heading in velocity from start to end
        velocity = (speed / distance) * direction

        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity -= self.velocity

        # Time to get from start to end
        t = distance / speed

        # The new end point takes the same time to reach, but at a new offset heading
        endPoint = startPoint + velocity * t

        # Given the start time, this line will have moved.  Alternately, we offset the start and end points in the
        # opposite direction
        offset = -self.velocity * startTime
        return self.checkLineIntersection(startPoint + offset, endPoint + offset)
