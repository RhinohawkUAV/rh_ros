from render.drawables import DrawablePolygon


class NoFlyZone(DrawablePolygon):
    def __init__(self, points, velocity):
        DrawablePolygon.__init__(self, points, fill="", outline="blue", width=5.0)
        self.velocity = velocity

    def blocksLineOfSight(self, line):
        return line.crosses(self.polygon) or line.within(self.polygon)
