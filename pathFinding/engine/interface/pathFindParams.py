from constants import DEFAULT_NFZ_BUFFER_WIDTH, DEFAULT_WAYPOINT_ACCEPTANCE_RADII, \
    DEFAULT_NFZ_TARGET_OFFSET, DEFAULT_VERTEX_HEURISTIC_MULTIPLIER


class PathFindParams:

    def __init__(self, waypointAcceptanceRadii=DEFAULT_WAYPOINT_ACCEPTANCE_RADII,
                       nfzBufferWidth=DEFAULT_NFZ_BUFFER_WIDTH,
                       nfzTargetOffset=DEFAULT_NFZ_TARGET_OFFSET,
                       vertexHeuristicMultiplier=DEFAULT_VERTEX_HEURISTIC_MULTIPLIER):
        self.waypointAcceptanceRadii = waypointAcceptanceRadii
        self.nfzBufferWidth = nfzBufferWidth
        self.nfzTargetOffset = nfzTargetOffset
        self.vertexHeuristicMultiplier = vertexHeuristicMultiplier


DEFAULT_PARAMS = PathFindParams()
