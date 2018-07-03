from constants import DEFAULT_NFZ_BUFFER_SIZE, DEFAULT_WAYPOINT_ACCEPTANCE_RADII


class PathFindParams:

    def __init__(self, waypointAcceptanceRadii, nfzBufferSize):
        self.waypointAcceptanceRadii = waypointAcceptanceRadii
        self.nfzBufferSize = nfzBufferSize


DEFAULT_PARAMS = PathFindParams(DEFAULT_WAYPOINT_ACCEPTANCE_RADII, DEFAULT_NFZ_BUFFER_SIZE)
