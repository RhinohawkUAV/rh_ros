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


def fromJSONDict(objDict):
    return DynamicNoFlyZone(objDict["center"], objDict["radius"], objDict["velocity"], objDict["ID"])
