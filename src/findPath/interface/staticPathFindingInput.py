from typing import Sequence

from noFlyZoneInput import NoFlyZoneInput


class StaticPathFindingInput:
    """
    Static input, for the path-finding problem.  This given once to the path-finder on creation.  This will be used to
    perform initial setup and determine an efficient waypoint ordering.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, startPosition, wayPoints, boundary, staticNoFlyZones):
        # type: (Sequence, [Sequence], [Sequence], [NoFlyZoneInput]) -> None

        # The take off position of Rhinohawk
        self._startPosition = startPosition

        # A sequence of waypoints
        self._wayPoints = wayPoints

        # A Boundary object
        self._boundary = boundary

        # A sequence of NoFlyZoneInput objects
        self._staticNoFlyZones = staticNoFlyZones
