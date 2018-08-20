from constants import DEFAULT_NFZ_BUFFER_WIDTH, DEFAULT_WAYPOINT_ACCEPTANCE_RADII, \
    DEFAULT_NFZ_TARGET_OFFSET


class PathFindParams:

    def __init__(self, waypointAcceptanceRadii, nfzBufferWidth, nfzTargetOffset):
        self.waypointAcceptanceRadii = waypointAcceptanceRadii
        self.nfzBufferWidth = nfzBufferWidth
        self.nfzTargetOffset = nfzTargetOffset


DEFAULT_PARAMS = PathFindParams(DEFAULT_WAYPOINT_ACCEPTANCE_RADII, DEFAULT_NFZ_BUFFER_WIDTH, DEFAULT_NFZ_TARGET_OFFSET)
