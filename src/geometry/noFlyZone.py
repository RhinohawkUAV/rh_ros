import Tkinter as tk
from Tkinter import Canvas

from shapely.geometry import Polygon

from render.Drawable import Drawable
from render.drawables import DrawablePolygon, DrawableLine


class NoFlyZone(Drawable):
    def __init__(self, vertices, velocity):
        self.vertices = vertices
        self.velocity = velocity
        self.time = 0.0
        self.fill = "red"

    def blocksLineOfSight(self, line):
        polygon = Polygon(self.vertices)
        return line.crosses(polygon) or line.within(polygon)

    def findFuturePoints(self, startPosition, speed):
        """Given the startPosition a speed of travel and the velocity of the NFZ,
        find the location in the future, where each point, of the NFZ will be intersected."""
        return self.vertices

    def blocksLineOfSightWithTime(self, line):
        polygon = Polygon(self.vertices)
        return line.crosses(polygon) or line.within(polygon)

    def draw(self, canvas):
        # type: (Canvas) -> None
        futureVertices = []
        for vertex in self.vertices:
            futureVertices.append((vertex[0] + self.velocity[0] * self.time, vertex[1] + self.velocity[1] * self.time))
        DrawablePolygon(futureVertices, fill=self.fill).draw(canvas)
        if self.velocity[0] != 0 or self.velocity[1] != 0:
            x1 = self.vertices[0][0]
            y1 = self.vertices[0][1]
            x2 = self.vertices[0][0] + self.velocity[0] * 4.0
            y2 = self.vertices[0][1] + self.velocity[1] * 4.0

            DrawableLine(x1, y1, x2, y2, fill="black", arrow=tk.LAST).draw(canvas)


def findFuturePoint(point, velocity, start, speed):
    return (25, 25)
