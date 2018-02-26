from Tkinter import Canvas

from shapely.geometry import LineString
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
        visiblePoints = []
        self.addDrawable(DrawCircle(position[0], position[1], 1.0, fill="red"))

        lines = []  # type: List[LineString]
        for noFlyZone in self.noFlyZones:
            for vertex in noFlyZone.polygon.exterior.coords:
                lines.append(LineString([position, vertex]))

        for line in lines:
            visible = True
            for noFlyZone in self.noFlyZones:
                visible = not noFlyZone.blocksLineOfSight(line)
                if visible is False:
                    break

            drawLine = DrawLine(lineString=line)
            if visible:
                visiblePoints.append(line.coords[1])
            else:
                drawLine["dash"] = [2, 2]
            self.addDrawable(drawLine)

        return visiblePoints

    def draw(self,
             canvas  # type: Canvas
             ):
        """Must be called from GUI thread"""
        for drawable in self.drawables:
            drawable.draw(canvas)

        for noFlyZone in self.noFlyZones:
            coords = []
            for vertex in noFlyZone.polygon.exterior.coords:
                coords.append(vertex[0])
                coords.append(vertex[1])
            canvas.create_polygon(coords, fill="", outline="blue")

    def addDrawable(self, drawable):
        self.drawables.append(drawable)


class DrawLine(dict):
    """Used to hold information about a line to draw on a canvas.
    It holds the coordinates of the line and is also a dictionary passed to the create_line() method."""

    def __init__(self, x1=None, y1=None, x2=None, y2=None, lineString=None, **kwargs):
        dict.__init__(self, **kwargs)
        if lineString is None:
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2
        else:
            self.x1 = float(lineString.coords[0][0])
            self.y1 = float(lineString.coords[0][1])
            self.x2 = float(lineString.coords[1][0])
            self.y2 = float(lineString.coords[1][1])

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
