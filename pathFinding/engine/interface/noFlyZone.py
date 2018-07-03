from engine.geometry import calcs
import numpy as np


class NoFlyZone:
    """
    Defines the an individual no fly zone which is input to the path-finder.  Velocity, should be set to 0 for static
    NFZs.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, points, velocity, ID=0):
        self.points = np.array(points, np.double)

        # A list of 2-d _points, defining the no fly zone _boundary, in counter-clockwise order.
        # Enforce CW winding (normals face outward)
        if calcs.woundCCW(self.points):
            self.points = np.flipud(self.points)

        # A vector describing the velocity of the no fly zone.
        self.velocity = np.array(velocity, np.double)

        # TODO: Not clear if we will need this or not.  The purpose would be for tracking changes in DFNZs over time
        # for the
        # purposes of tracking an acceleration.  Not clear this would be fruitful or worthwhile.  For now we ignore this
        # and assume no history.
        self.ID = ID


def fromJSONDict(objDict):
    return NoFlyZone(objDict["points"], objDict["velocity"], objDict["ID"])
