import numpy as np
from typing import Sequence

from findPath.geometry import calcs
from noFlyZoneInput import NoFlyZoneInput


class InitialPathFindingInput:
    """
    Initial, one-time input, for the path-finding problem.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, boundaryPoints, noFlyZones):
        # type: ([Sequence], [NoFlyZoneInput]) -> None

        # Defines the boundary polygon (geo-fence) for the path-finding problem.  According to the rules this can be
        # max 18 sided, this will accept any number of sides.
        # TODO: Automatically convert point order
        # Must be in CCW order
        self.boundaryPoints = np.array(boundaryPoints, np.double)

        # A sequence of NoFlyZoneInput objects.  Not clear if rules allow dynamic NFZs can be announced initially,
        # but this allows for that case.
        self.noFlyZones = noFlyZones

    def calcBounds(self):
        return calcs.calcBounds(self.boundaryPoints)
