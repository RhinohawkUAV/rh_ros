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

    def isPositionVisible(self, eye, position):
        """Is the given position visible from eye?"""
        line = LineString([eye, position])
        for noFlyZone in self.noFlyZones:
            if noFlyZone.blocksLineOfSight(line):
                return False
        return True

    def findVisibleVertices(self, eye, end):
        """Find all vertices (including end) visible from eye"""
        visibleVertices = []
        for noFlyZone in self.noFlyZones:
            for polygonVertex in noFlyZone.vertices:
                if self.isPositionVisible(eye, polygonVertex):
                    visibleVertices.append(polygonVertex)
        if self.isPositionVisible(eye, end):
            visibleVertices.append(end)

        # Drawing
        for vertex in visibleVertices:
            drawLine = DrawableLine(eye[0], eye[1], vertex[0], vertex[1], fill="red")
            self.addDrawable(drawLine)

        return visibleVertices

    def findPath(self, start, end, renderer):
        self.graph = VertexBag()
        self.graph.updateVertex(start, 0)

        while self.graph.hasUnvisted():
            vertex = self.graph.getLowestCostUnvisted()

            if vertex.position is end:
                break

            visiblePoints = self.findVisibleVertices(vertex.position, end)

            for visiblePoint in visiblePoints:
                if not self.graph.beenVisited(visiblePoint):
                    x = vertex.position[0] - visiblePoint[0]
                    y = vertex.position[1] - visiblePoint[1]
                    distance = math.sqrt(x * x + y * y)
                    totalDistance = distance + vertex.shortestDistance
                    self.graph.updateVertex(visiblePoint, totalDistance)
                    self.graph.putEdge(vertex.position, visiblePoint, distance)

            self.addDrawable(DrawableCircle(vertex.position[0], vertex.position[1], 1.0, fill="green"))
            self.addDrawable(DrawableCircle(end[0], end[1], 1.0, fill="green"))
            renderCopy = copy.deepcopy(self)
            renderer.render(renderCopy)
            self.clearDrawables()
            time.sleep(1)

    def draw(self, canvas):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas)

        self.graph.draw(canvas)

        DrawGroup.draw(self, canvas)
