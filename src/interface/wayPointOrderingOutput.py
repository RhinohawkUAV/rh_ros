class WayPointOrderingOutput:
    """
    Output from solving the initial waypoint ordering problem.
    """

    def __init__(self, wayPointIndices):
        # type: ([int]) -> None

        # The order to visit the waypoints.  The index refers to the index for the way point in the input.
        self._wayPointIndices = wayPointIndices
