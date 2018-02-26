from shapely.geometry import Polygon


class NoFlyZone:
    def __init__(self, points, velocity):
        self.polygon = Polygon(points)
        self.velocity = velocity

    def blocksLineOfSight(self, line):
        return line.crosses(self.polygon)
