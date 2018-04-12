from typing import Sequence

from noFlyZoneInput import NoFlyZoneInput


class PointToPointInput:
    """
    Input for solving a point-to-point path-finding problem.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, startPosition, startVelocity, targetPoints, dynamicNoFlyZones):
        # type: (Sequence, Sequence, [Sequence], [NoFlyZoneInput]) -> None

        # The current position of Rhinohawk
        self.startPosition = startPosition

        # The current velocity of Rhinohawk
        self.startVelocity = startVelocity

        # TODO: This is not supported yet and may not be necessary, we'll decide how fancy we want to get
        # Allows for one or more ordered targets to be passed through, in order.
        self.targetPoints = targetPoints

        # The current state of all dynamic no fly zones
        self.dynamicNoFlyZones = dynamicNoFlyZones
