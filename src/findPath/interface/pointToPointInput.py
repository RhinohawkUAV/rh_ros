from typing import Sequence

from noFlyZoneInput import NoFlyZoneInput


class PointToPointInput:
    """
    Input for solving an individual point-to-point path-finding problem.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, startPosition, startVelocity, dynamicNoFlyZones):
        # type: (Sequence, Sequence, [NoFlyZoneInput]) -> None

        # The current position of Rhinohawk
        self._startPosition = startPosition

        # The current velocity of Rhinohawk
        self._startVelocity = startVelocity

        # The current state of all dynamic no fly zones
        self._dynamicNoFlyZones = dynamicNoFlyZones
