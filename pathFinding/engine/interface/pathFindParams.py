from constants import DEFAULT_NFZ_BUFFER_WIDTH, DEFAULT_WAYPOINT_ACCEPTANCE_RADII, \
    DEFAULT_NFZ_TARGET_OFFSET, DEFAULT_VERTEX_HEURISTIC_MULTIPLIER, \
    DEFAULT_MAXIMUM_TIME


class PathFindParams:

    # TODO: Integrate maximum time into the rest of system and ROS
    def __init__(self, waypointAcceptanceRadii=DEFAULT_WAYPOINT_ACCEPTANCE_RADII,
                       nfzBufferWidth=DEFAULT_NFZ_BUFFER_WIDTH,
                       nfzTargetOffset=DEFAULT_NFZ_TARGET_OFFSET,
                       vertexHeuristicMultiplier=DEFAULT_VERTEX_HEURISTIC_MULTIPLIER,
                       maximumTime=DEFAULT_MAXIMUM_TIME):
        self.waypointAcceptanceRadii = waypointAcceptanceRadii
        self.nfzBufferWidth = nfzBufferWidth
        self.nfzTargetOffset = nfzTargetOffset
        self.vertexHeuristicMultiplier = vertexHeuristicMultiplier
        self.maximumTime = maximumTime


DEFAULT_PARAMS = PathFindParams()


def fromDict(inputDict):
    """
    Allows loading params from disc, accounting for its changing nature (new parameters added over time).
    """    
    args = [inputDict["waypointAcceptanceRadii"],
                            inputDict["nfzBufferWidth"],
                            inputDict["nfzTargetOffset"],
                            inputDict["vertexHeuristicMultiplier"]]
    if inputDict.has_key("maximumTime"):
        args.append(inputDict["maximumTime"])
    
    return PathFindParams(*args)

