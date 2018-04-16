from typing import Sequence


class NoFlyZoneInput:
    """
    Defines the an individual no fly zone which is input to the path-finder.  Velocity, should be set to 0 for static
    NFZs.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, points, velocity, ID=0):
        # type: ([Sequence],Sequence) -> None

        # A list of 2-d _points, defining the no fly zone _boundary, in counter-clockwise order.
        # TODO: Must be in clockwise order.  Must automate that.
        self.points = points

        # A vector describing the velocity of the no fly zone.
        self.velocity = velocity

        # TODO: Not clear if we will need this or not.  The purpose would be for tracking changes in DFNZs over time
        # for the
        # purposes of tracking an acceleration.  Not clear this would be fruitful or worthwhile.  For now we ignore this
        # and assume no history.
        self.ID = ID


def fromJSONDict(objDict):
    return NoFlyZoneInput(objDict["points"], objDict["velocity"], objDict["ID"])
