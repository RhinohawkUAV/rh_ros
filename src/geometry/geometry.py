from Tkinter import Canvas

from shapely.geometry import LineString
from typing import List

from noFlyZone import NoFlyZone
from render.drawGroup import DrawGroup
from render.drawables import DrawableCircle, DrawableLine


class Geometry(DrawGroup):
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.
    """

    def __init__(self, noFlyZones):
        # type: (List[NoFlyZone]) -> None

        DrawGroup.__init__(self)
        self.noFlyZones = noFlyZones

    def findVisibleVertices(self, position):
        visiblePoints = []
        self.addDrawable(DrawableCircle(position[0], position[1], 1.0, fill="red"))

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

            drawLine = DrawableLine(lineString=line)
            if visible:
                visiblePoints.append(line.coords[1])
            else:
                drawLine["dash"] = [2, 2]
            self.addDrawable(drawLine)

        return visiblePoints

    def draw(self, canvas):
        # type: (Canvas)->None
        """Must be called from GUI thread"""
        DrawGroup.draw(self, canvas)

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas)
