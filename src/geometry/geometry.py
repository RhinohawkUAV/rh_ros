import Tkinter as tk
from Tkinter import Canvas

from sympy import Segment2D
from typing import Any
from typing import List

from noFlyZone import NoFlyZone


class Geometry:
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.

    """

    def __init__(self, noFlyZones):
        # type: (List[NoFlyZone]) -> None

        self.noFlyZones = noFlyZones

        # Used to render segments on canvas for visualization/debugging
        self.drawables = []  # type: List[Any]

    def findVisibleVertices(self, position):
        self.addDrawable(DrawCircle(float(position.x), float(position.y), 1.0, fill="red"))

        lines = []  # type: List[Segment2D]
        for noFlyZone in self.noFlyZones:
            for vertex in noFlyZone.polygon.vertices:
                lines.append(Segment2D(position, vertex))

        visiblePoints = []
        for line in lines:
            visible = True
            for noFlyZone in self.noFlyZones:
                visible = not noFlyZone.blocksLineOfSight(line)
                if visible is False:
                    break

            drawLine = DrawLine(segment=line)
            if visible:
                visiblePoints.append(line.p2)
            else:
                drawLine["dash"] = [2, 2]
            self.addDrawable(drawLine)

        return visiblePoints

    def draw(self,
             canvas  # type: Canvas
             ):
        """Must be called from GUI thread"""
        canvas.delete(tk.ALL)
        for drawable in self.drawables:
            drawable.draw(canvas)

        for noFlyZone in self.noFlyZones:
            coords = []
            for vertex in noFlyZone.polygon.vertices:
                coords.append(float(vertex.x))
                coords.append(float(vertex.y))
            canvas.create_polygon(coords, fill="", outline="blue")

    def addDrawable(self, drawable):
        self.drawables.append(drawable)


class DrawLine(dict):
    """Used to hold information about a line to draw on a canvas.
    It holds the coordinates of the line and is also a dictionary passed to the create_line() method."""

    def __init__(self, x1=None, y1=None, x2=None, y2=None, segment=None, **kwargs):
        dict.__init__(self, **kwargs)
        if segment is None:
            self.x1 = x1
            self.x2 = x2
            self.y1 = y1
            self.y2 = y2
        else:
            self.x1 = float(segment.p1.x)
            self.x2 = float(segment.p2.x)
            self.y1 = float(segment.p1.y)
            self.y2 = float(segment.p2.y)

    def draw(self,
             canvas  # type: Canvas
             ):
        canvas.create_line(self.x1, self.y1, self.x2, self.y2, **self)


class DrawCircle(dict):
    """Used to hold information about a circle to draw on a canvas.
    It holds the coordinates of the circle and is also a dictionary passed to the create_oval() method."""

    def __init__(self, x=None, y=None, radius=None, **kwargs):
        dict.__init__(self, **kwargs)
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self,
             canvas  # type: Canvas
             ):
        canvas.create_oval(self.x - self.radius, self.y - self.radius, self.x + self.radius, self.y + self.radius,
                           **self)
