from sympy.geometry import Polygon
from sympy.geometry import Segment2D


class NoFlyZone:
    def __init__(self, points, velocity):
        self.polygon = Polygon(*points)
        self.velocity = velocity

    def getPointsOfInterest(self, position):
        points = []
        for vertex in self.polygon.vertices:
            line = Segment2D(position, vertex)
            points.append(vertex)
        return points
