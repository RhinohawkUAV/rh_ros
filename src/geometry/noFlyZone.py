from Tkinter import Canvas

from shapely.geometry import Polygon

from render.Drawable import Drawable
from render.drawables import DrawablePolygon


class NoFlyZone(Drawable):
    def __init__(self, vertices, velocity):
        self.vertices = vertices
        self.velocity = velocity

    def blocksLineOfSight(self, line):
        polygon = Polygon(self.vertices)
        return line.crosses(polygon) or line.within(polygon)

    def draw(self, canvas):
        # type: (Canvas) -> None
        DrawablePolygon(self.vertices, fill="red").draw(canvas)
