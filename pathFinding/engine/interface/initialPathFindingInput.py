import numpy as np
from typing import Sequence

from engine.geometry import calcs
from engine.interface import noFlyZoneInput
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
        # Enforce CW winding (normals face inward)

        self.boundaryPoints = np.array(boundaryPoints, np.double)
        if not calcs.woundCCW(self.boundaryPoints):
            self.boundaryPoints = list(reversed(boundaryPoints))
            # TODO: Test that this really flips properly

        # A sequence of NoFlyZoneInput objects.  Not clear if rules allow dynamic NFZs can be announced initially,
        # but this allows for that case.
        self.noFlyZones = noFlyZones

    def calcBounds(self):
        return calcs.calcBounds(self.boundaryPoints)


def fromJSONDict(objDict):
    noFlyZones = []
    for noFlyDict in objDict["noFlyZones"]:
        noFlyZones.append(noFlyZoneInput.fromJSONDict(noFlyDict))
    return InitialPathFindingInput(objDict["boundaryPoints"], noFlyZones)
