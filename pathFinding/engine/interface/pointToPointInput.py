from typing import Sequence


class PointToPointInput:
    """
    Input for solving a point-to-point path-finding problem.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, startPosition, startVelocity, targetPoints):
        # type: (Sequence, Sequence, [Sequence]) -> None

        # The current position of Rhinohawk
        self.startPosition = startPosition

        # The current velocity of Rhinohawk
        self.startVelocity = startVelocity

        # TODO: This is not supported yet and may not be necessary, we'll decide how fancy we want to get
        # Plan a route from start position, through these targets, in order
        self.targetPoints = targetPoints


def fromJSONDict(objDict):
    return PointToPointInput(objDict["startPosition"], objDict["startVelocity"], objDict["targetPoints"])
