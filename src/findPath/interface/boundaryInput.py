from typing import Sequence


class BoundaryInput:
    """
    Defines the _boundary (geo-fence) for the path-finding problem.  According to the rules this can be max 18 sided, but
    this will accept any number of sizes.

    All coordinates/vectors should be in a consistent, cartesian, 2-d coordinate system.
    """

    def __init__(self, points):
        # type: ([Sequence])->None

        # A list of _points defining the _boundary, in CCW order.
        # TODO: CCW order not currently guaranteed.
        self._points = points
