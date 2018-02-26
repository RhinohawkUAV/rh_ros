import copy
import math
import time
from Tkinter import Canvas

from shapely.geometry import LineString
from typing import List

from graph.graph import VertexBag
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

    def findVisibleVertices(self, position, end):
        visiblePoints = []

        originalPoints = []
        lines = []  # type: List[LineString]
        for noFlyZone in self.noFlyZones:
            for polgonVertex in noFlyZone.polygon.exterior.coords:
                lines.append(LineString([position, polgonVertex]))
                originalPoints.append(polgonVertex)
        lines.append(LineString([position, end]))
        originalPoints.append(end)

        i = 0
        while i < len(lines):
            line = lines[i]
            visible = True
            for noFlyZone in self.noFlyZones:
                visible = not noFlyZone.blocksLineOfSight(line)
                if visible is False:
                    break

            drawLine = DrawableLine(lineString=line, fill="red")
            if visible:
                visiblePoints.append(originalPoints[i])
            else:
                drawLine["dash"] = [2, 2]
            self.addDrawable(drawLine)
            i += 1
        return visiblePoints

    def findPath(self, start, end, renderer):
        # self.addDrawable(DrawableCircle(start[0], start[1], 1.0, fill="green"))
        self.graph = VertexBag()
        self.graph.updateVertex(start, 0)

        while self.graph.hasUnvisted():
            vertex = self.graph.getLowestCostUnvisted()

            if vertex.position is end:
                break

            self.addDrawable(DrawableCircle(vertex.position[0], vertex.position[1], 1.0, fill="green"))
            self.addDrawable(DrawableCircle(end[0], end[1], 1.0, fill="green"))
            visiblePoints = self.findVisibleVertices(vertex.position, end)

            for visiblePoint in visiblePoints:
                if not self.graph.beenVisited(visiblePoint):
                    x = vertex.position[0] - visiblePoint[0]
                    y = vertex.position[1] - visiblePoint[1]
                    distance = math.sqrt(x * x + y * y)
                    totalDistance = distance + vertex.shortestDistance
                    self.graph.updateVertex(visiblePoint, totalDistance)
                    self.graph.putEdge(vertex.position, visiblePoint, distance)
            renderCopy = copy.deepcopy(self)
            renderer.render(renderCopy)
            self.clearDrawables()
            time.sleep(1)

    def draw(self, canvas):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas)

        DrawGroup.draw(self, canvas)

        self.graph.draw(canvas)
