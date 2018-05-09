from engine.geometry import LineSegment


class LinearPath:
    """
    Represents a path from startPoint to endPoint at a given speed.
    """

    def __init__(self, startPoint, endPoint, speed):
        self._startPoint = startPoint
        self._endPoint = endPoint
        self._speed = speed
        self._lineSegment = LineSegment(self._startPoint, self._endPoint)

    def intersectsWithObstacleLine(self, obstacleLine):
        obstacleLine.checkPathIntersectsLine(self._startPoint, self._endPoint, self._speed)
