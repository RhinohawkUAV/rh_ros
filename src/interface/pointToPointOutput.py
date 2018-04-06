from typing import Sequence


class PointToPointOutput:
    """
    Output from solving an individual point-to-point path-finding problem.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, positions, velocities):
        # type: ([Sequence], [Sequence]) -> None

        # A list of of positions which should be visited
        self._positions = positions

        # A corresponding list of velocity vectors for each position
        # TODO: This should ultimately be a list of speeds for the flight controller, but intermediate translation code may want velocity info.
        self._velocities = velocities
