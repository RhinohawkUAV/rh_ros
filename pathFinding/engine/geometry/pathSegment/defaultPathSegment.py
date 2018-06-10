from engine.geometry import PathSegment


class DefaultPathSegment(PathSegment):
    def __init__(self, startTime, elapsedTime, lineEndPoint, endVelocity):
        PathSegment.__init__(self, startTime, elapsedTime, lineEndPoint, endVelocity)

    def intersectsObstacleLine(self, startTime, obstacleLine):
        return False
