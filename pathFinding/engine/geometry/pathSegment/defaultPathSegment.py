from engine.geometry import PathSegment


class DefaultPathSegment(PathSegment):
    def __init__(self, time, endPoint, endVelocity):
        PathSegment.__init__(self, time, endPoint, endVelocity)

    def intersectsLine(self, obstacleLine, obstacleLineVelocity):
        return False
