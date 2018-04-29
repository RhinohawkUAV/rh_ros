from engine.geometry import LineSegment


class LinearPath:
    """
    Represents a path from start to end at a given speed.
    """

    def __init__(self, start, end, speed):
        self._startPoint = start
        self._endPoint = end
        self._speed = speed
        self._lineSegment = LineSegment(self._startPoint, self._endPoint)

    def intersectsWithObstacleLine(self, obstacleLine):
        obstacleLine.checkPathIntersectsLine(self._startPoint, self._endPoint, self._speed)
