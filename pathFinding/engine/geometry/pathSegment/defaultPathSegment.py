from engine.geometry import PathSegment


class DefaultPathSegment(PathSegment):
    def __init__(self, time, endPoint, endVelocity):
        PathSegment.__init__(self, time, endPoint, endVelocity)

    def intersectsObstacleLine(self, startTime, obstacleLine):
        return False
