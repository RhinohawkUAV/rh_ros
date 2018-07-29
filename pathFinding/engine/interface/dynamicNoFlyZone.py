from engine.geometry import calcs
import numpy as np


class DynamicNoFlyZone:
    """
    Defines the an individual dynamic no fly zone which is input to the path-finder.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, center, radius, velocity, ID=0):
        self.center = np.array(center, np.double)
        self.radius = radius

        # A vector describing the velocity of the no fly zone.
        self.velocity = np.array(velocity, np.double)

        # TODO: Not clear if we will need this or not.  The purpose would be for tracking changes in DFNZs over time
        # for the
        # purposes of tracking an acceleration.  Not clear this would be fruitful or worthwhile.  For now we ignore this
        # and assume no history.
        self.ID = ID

    def checkPathIntersection(self, startTime, startPoint, endPoint, speed):
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

        # Given the start time, this dnfz will have moved.  Alternately, we offset the start and end points in the
        # opposite direction
        offset = -self.velocity * startTime
        return calcs.lineSegmentCircleIntersect(startPoint + offset, endPoint + offset, self.center, self.radius)


def fromJSONDict(objDict):
    return DynamicNoFlyZone(objDict["center"], objDict["radius"], objDict["velocity"], objDict["ID"])
